#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"

namespace ogmpp_planners {
    namespace prms {
        std::map<std::string, double> UniformSampling::_fixParameters(std::map<std::string, double> parameters) {
            std::map<std::string, double> p, temp_map;
            double temp;

            // Add default values
            temp_map.insert(std::pair<std::string, double>("uniform_sampling_step", 0.4));
            temp_map.insert(std::pair<std::string, double>("uniform_minimum_distance_from_wall", 0.3));

            // Check all for yaml and on-demand parameters
            for (std::map<std::string, double>::iterator it = temp_map.begin(); it != temp_map.end(); it++) {
                p.insert(std::pair<std::string, double>(it->first, it->second));
                if (_nh.hasParam(it->first)) {
                    _nh.getParam(it->first, temp);
                    p[it->first] = temp;
                }
                if (parameters.find(it->first) != parameters.end()) {
                    p[it->first] = parameters[it->first];
                }
            }
            return p;
        }

        /**
         * @brief Creates the uniform sampling graph
         * @param map [ogmpp_map_loader&] The map
         * @param begin [ogmpp_graph::Cell] The starting cell
         * @param end [ogmpp_graph::Cell] The ending cell
         */
        ogmpp_graph::Graph
        UniformSampling::_createGraph(ogmpp_map_loader::Map &map, ogmpp_graph::Cell begin, ogmpp_graph::Cell end,
                                      std::map<std::string, double> parameters) {
            std::pair<unsigned int, unsigned int> size = map.getMapSize();
            unsigned int w = size.first;
            unsigned int h = size.second;
            std::map<std::string, double> p = _fixParameters(parameters);
            double step_f = p["uniform_sampling_step"] / map.getResolution();
            double min_dist_from_wall_f = p["uniform_minimum_distance_from_wall"] / map.getResolution();

            ogmpp_graph::Graph _g;
            _g.clean();
            _g = ogmpp_graph::Graph(map.getResolution());

            int min_dist_from_wall = int(min_dist_from_wall_f + 0.5);
            int step = int(step_f + 0.5);
            long x = 0;
            while (x < w) {
                long y = 0;
                while (y < h) {
                    if (map.isUnoccupied(x, y) && !map.isUnknown(x, y) &&
                        map.getDistanceTransformation(x, y) > min_dist_from_wall) {
                        _g.addNode(ogmpp_graph::Cell(x, y));
                        // Make connections
                        if (x - step >= 0 && y - step >= 0 &&
                            _g.getNode(ogmpp_graph::Cell(x - step, y - step)) != NULL) {
                            bool throughWall = false;
                            for (int i = 1; i < step && !throughWall; i++) {
                                if (!map.isUnoccupied(x - i, y - i)) {
                                    throughWall = true;
                                    ROS_WARN_STREAM("Discarding wall neighbor!");
                                }
                            }
                            if (!throughWall) {
                                _g.makeNeighbor(ogmpp_graph::Cell(x, y), ogmpp_graph::Cell(x - step, y - step));
                            }
                        }
                        if (y - step >= 0 && _g.getNode(ogmpp_graph::Cell(x, y - step)) != NULL) {
                            bool throughWall = false;
                            for (int i = 1; i < step && !throughWall; i++) {
                                if (!map.isUnoccupied(x, y - i)) {
                                    throughWall = true;
                                    ROS_WARN_STREAM("Discarding wall neighbor!");
                                }
                            }
                            if (!throughWall) {
                                _g.makeNeighbor(ogmpp_graph::Cell(x, y), ogmpp_graph::Cell(x, y - step));
                            }
                        }
                        if (x - step >= 0 && y + step < h &&
                            _g.getNode(ogmpp_graph::Cell(x - step, y + step)) != NULL) {
                            bool throughWall = false;
                            for (int i = 1; i < step && !throughWall; i++) {
                                if (!map.isUnoccupied(x - i, y + i)) {
                                    throughWall = true;
                                    ROS_WARN_STREAM("Discarding wall neighbor!");
                                }
                            }
                            if (!throughWall) {
                                _g.makeNeighbor(ogmpp_graph::Cell(x, y), ogmpp_graph::Cell(x - step, y + step));
                            }
                        }
                        if (x - step >= 0 && _g.getNode(ogmpp_graph::Cell(x - step, y)) != NULL) {
                            bool throughWall = false;
                            for (int i = 1; i < step && !throughWall; i++) {
                                if (!map.isUnoccupied(x - i, y)) {
                                    throughWall = true;
                                    ROS_WARN_STREAM("Discarding wall neighbor!");
                                }
                            }
                            if (!throughWall) {
                                _g.makeNeighbor(ogmpp_graph::Cell(x, y), ogmpp_graph::Cell(x - step, y));
                            }
                        }
                    }
                    y += step;
                }
                x += step;
            }

            // Add robot and goal poses
            if (!map.isUnoccupied(begin.x, begin.y) || !map.isUnoccupied(end.x, end.y) ||
                map.isUnknown(begin.x, begin.y) || map.isUnknown(end.x, end.y)) {
                ROS_ERROR_STREAM("ogmpp_uniform_sampling: Robot or goal pose is not unoccupied");
            }

            _g.addNode(begin);
            _g.addNode(end);
            ogmpp_graph::Cell base_cell_begin = ogmpp_graph::Cell((begin.x / step) * step, (begin.y / step) * step);
            for (int i = -2; i <= 2; i++) {
                for (int j = -2; j <= 2; j++) {
                    long xx = base_cell_begin.x + i * step;
                    long yy = base_cell_begin.y + j * step;
                    if (xx < 0 || xx >= w || yy < 0 || yy >= h)
                        continue;
                    if (_g.getNode(ogmpp_graph::Cell(xx, yy)) != NULL) {
                        _g.makeNeighbor(ogmpp_graph::Cell(xx, yy), begin);
                    }
                }
            }

            ogmpp_graph::Cell base_cell_end = ogmpp_graph::Cell((end.x / step) * step, (end.y / step) * step);
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    long xx = base_cell_end.x + i * step;
                    long yy = base_cell_end.y + j * step;
                    if (xx < 0 || xx >= w || yy < 0 || yy >= h)
                        continue;
                    if (_g.getNode(ogmpp_graph::Cell(xx, yy)) != NULL) {
                        _g.makeNeighbor(ogmpp_graph::Cell(xx, yy), end);
                    }
                }
            }
            return _g;
        }
    }
}
