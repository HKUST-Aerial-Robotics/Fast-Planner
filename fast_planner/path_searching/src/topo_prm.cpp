/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include <path_searching/topo_prm.h>
#include <thread>

namespace fast_planner {
TopologyPRM::TopologyPRM(/* args */) {}

TopologyPRM::~TopologyPRM() {}

void TopologyPRM::init(ros::NodeHandle& nh) {
  graph_.clear();
  eng_ = default_random_engine(rd_());
  rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

  // init parameter
  nh.param("topo_prm/sample_inflate_x", sample_inflate_(0), -1.0);
  nh.param("topo_prm/sample_inflate_y", sample_inflate_(1), -1.0);
  nh.param("topo_prm/sample_inflate_z", sample_inflate_(2), -1.0);
  nh.param("topo_prm/clearance", clearance_, -1.0);
  nh.param("topo_prm/short_cut_num", short_cut_num_, -1);
  nh.param("topo_prm/reserve_num", reserve_num_, -1);
  nh.param("topo_prm/ratio_to_short", ratio_to_short_, -1.0);
  nh.param("topo_prm/max_sample_num", max_sample_num_, -1);
  nh.param("topo_prm/max_sample_time", max_sample_time_, -1.0);
  nh.param("topo_prm/max_raw_path", max_raw_path_, -1);
  nh.param("topo_prm/max_raw_path2", max_raw_path2_, -1);
  nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, false);
  resolution_ = edt_environment_->sdf_map_->getResolution();
  offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - edt_environment_->sdf_map_->getOrigin() / resolution_;

  for (int i = 0; i < max_raw_path_; ++i) {
    casters_.push_back(RayCaster());
  }
}

void TopologyPRM::findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end,
                                vector<Eigen::Vector3d> start_pts, vector<Eigen::Vector3d> end_pts,
                                list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& raw_paths,
                                vector<vector<Eigen::Vector3d>>& filtered_paths,
                                vector<vector<Eigen::Vector3d>>& select_paths) {
  ros::Time t1, t2;

  double graph_time, search_time, short_time, prune_time, select_time;
  /* ---------- create the topo graph ---------- */
  t1 = ros::Time::now();

  start_pts_ = start_pts;
  end_pts_ = end_pts;

  graph = createGraph(start, end);

  graph_time = (ros::Time::now() - t1).toSec();

  /* ---------- search paths in the graph ---------- */
  t1 = ros::Time::now();

  raw_paths = searchPaths();

  search_time = (ros::Time::now() - t1).toSec();

  /* ---------- path shortening ---------- */
  // for parallel, save result in short_paths_
  t1 = ros::Time::now();

  shortcutPaths();

  short_time = (ros::Time::now() - t1).toSec();

  /* ---------- prune equivalent paths ---------- */
  t1 = ros::Time::now();

  filtered_paths = pruneEquivalent(short_paths_);

  prune_time = (ros::Time::now() - t1).toSec();
  // cout << "prune: " << (t2 - t1).toSec() << endl;

  /* ---------- select N shortest paths ---------- */
  t1 = ros::Time::now();

  select_paths = selectShortPaths(filtered_paths, 1);

  select_time = (ros::Time::now() - t1).toSec();

  final_paths_ = select_paths;

  double total_time = graph_time + search_time + short_time + prune_time + select_time;

  std::cout << "\n[Topo]: total time: " << total_time << ", graph: " << graph_time
            << ", search: " << search_time << ", short: " << short_time << ", prune: " << prune_time
            << ", select: " << select_time << std::endl;
}

list<GraphNode::Ptr> TopologyPRM::createGraph(Eigen::Vector3d start, Eigen::Vector3d end) {
  // std::cout << "[Topo]: searching----------------------" << std::endl;

  /* init the start, end and sample region */
  graph_.clear();
  // collis_.clear();

  GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
  GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

  graph_.push_back(start_node);
  graph_.push_back(end_node);

  // sample region
  sample_r_(0) = 0.5 * (end - start).norm() + sample_inflate_(0);
  sample_r_(1) = sample_inflate_(1);
  sample_r_(2) = sample_inflate_(2);

  // transformation
  translation_ = 0.5 * (start + end);

  Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
  xtf = (end - translation_).normalized();
  ytf = xtf.cross(downward).normalized();
  ztf = xtf.cross(ytf);

  rotation_.col(0) = xtf;
  rotation_.col(1) = ytf;
  rotation_.col(2) = ztf;

  int node_id = 1;

  /* ---------- main loop ---------- */
  int sample_num = 0;
  double sample_time = 0.0;
  Eigen::Vector3d pt;
  ros::Time t1, t2;
  while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
    t1 = ros::Time::now();

    pt = getSample();
    ++sample_num;
    double dist;
    Eigen::Vector3d grad;
    // edt_environment_->evaluateEDTWithGrad(pt, -1.0, dist, grad);
    dist = edt_environment_->evaluateCoarseEDT(pt, -1.0);
    if (dist <= clearance_) {
      sample_time += (ros::Time::now() - t1).toSec();
      continue;
    }

    /* find visible guard */
    vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);
    if (visib_guards.size() == 0) {
      GraphNode::Ptr guard = GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
      graph_.push_back(guard);
    } else if (visib_guards.size() == 2) {
      /* try adding new connection between two guard */
      // vector<pair<GraphNode::Ptr, GraphNode::Ptr>> sort_guards =
      // sortVisibGuard(visib_guards);
      bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
      if (!need_connect) {
        sample_time += (ros::Time::now() - t1).toSec();
        continue;
      }
      // new useful connection needed, add new connector
      GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
      graph_.push_back(connector);

      // connect guards
      visib_guards[0]->neighbors_.push_back(connector);
      visib_guards[1]->neighbors_.push_back(connector);

      connector->neighbors_.push_back(visib_guards[0]);
      connector->neighbors_.push_back(visib_guards[1]);
    }

    sample_time += (ros::Time::now() - t1).toSec();
  }

  /* print record */
  std::cout << "[Topo]: sample num: " << sample_num;

  pruneGraph();
  // std::cout << "[Topo]: node num: " << graph_.size() << std::endl;

  return graph_;
  // return searchPaths(start_node, end_node);
}

vector<GraphNode::Ptr> TopologyPRM::findVisibGuard(Eigen::Vector3d pt) {
  vector<GraphNode::Ptr> visib_guards;
  Eigen::Vector3d pc;

  int visib_num = 0;

  /* find visible GUARD from pt */
  for (list<GraphNode::Ptr>::iterator iter = graph_.begin(); iter != graph_.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Connector) continue;

    if (lineVisib(pt, (*iter)->pos_, resolution_, pc)) {
      visib_guards.push_back((*iter));
      ++visib_num;
      if (visib_num > 2) break;
    }
  }

  return visib_guards;
}

bool TopologyPRM::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt) {
  vector<Eigen::Vector3d> path1(3), path2(3);
  path1[0] = g1->pos_;
  path1[1] = pt;
  path1[2] = g2->pos_;

  path2[0] = g1->pos_;
  path2[2] = g2->pos_;

  vector<Eigen::Vector3d> connect_pts;
  bool has_connect = false;
  for (int i = 0; i < g1->neighbors_.size(); ++i) {
    for (int j = 0; j < g2->neighbors_.size(); ++j) {
      if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
        path2[1] = g1->neighbors_[i]->pos_;
        bool same_topo = sameTopoPath(path1, path2, 0.0);
        if (same_topo) {
          // get shorter connection ?
          if (pathLength(path1) < pathLength(path2)) {
            g1->neighbors_[i]->pos_ = pt;
            // ROS_WARN("shorter!");
          }
          return false;
        }
      }
    }
  }
  return true;
}

Eigen::Vector3d TopologyPRM::getSample() {
  /* sampling */
  Eigen::Vector3d pt;
  pt(0) = rand_pos_(eng_) * sample_r_(0);
  pt(1) = rand_pos_(eng_) * sample_r_(1);
  pt(2) = rand_pos_(eng_) * sample_r_(2);

  pt = rotation_ * pt + translation_;

  return pt;
}

bool TopologyPRM::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                            Eigen::Vector3d& pc, int caster_id) {
  Eigen::Vector3d ray_pt;
  Eigen::Vector3i pt_id;
  double dist;

  casters_[caster_id].setInput(p1 / resolution_, p2 / resolution_);
  while (casters_[caster_id].step(ray_pt)) {
    pt_id(0) = ray_pt(0) + offset_(0);
    pt_id(1) = ray_pt(1) + offset_(1);
    pt_id(2) = ray_pt(2) + offset_(2);
    dist = edt_environment_->sdf_map_->getDistance(pt_id);
    if (dist <= thresh) {
      edt_environment_->sdf_map_->indexToPos(pt_id, pc);
      return false;
    }
  }
  return true;
}

void TopologyPRM::pruneGraph() {
  /* prune useless node */
  if (graph_.size() > 2) {
    for (list<GraphNode::Ptr>::iterator iter1 = graph_.begin();
         iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
      if ((*iter1)->id_ <= 1) continue;

      /* core */
      // std::cout << "id: " << (*iter1)->id_ << std::endl;
      if ((*iter1)->neighbors_.size() <= 1) {
        // delete this node from others' neighbor
        for (list<GraphNode::Ptr>::iterator iter2 = graph_.begin(); iter2 != graph_.end(); ++iter2) {
          for (vector<GraphNode::Ptr>::iterator it_nb = (*iter2)->neighbors_.begin();
               it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
            if ((*it_nb)->id_ == (*iter1)->id_) {
              (*iter2)->neighbors_.erase(it_nb);
              break;
            }
          }
        }

        // delete this node from graph, restart checking
        graph_.erase(iter1);
        iter1 = graph_.begin();
      }
    }
  }
}

vector<vector<Eigen::Vector3d>> TopologyPRM::pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths) {
  vector<vector<Eigen::Vector3d>> pruned_paths;
  if (paths.size() < 1) return pruned_paths;

  /* ---------- prune topo equivalent path ---------- */
  // output: pruned_paths
  vector<int> exist_paths_id;
  exist_paths_id.push_back(0);

  for (int i = 1; i < paths.size(); ++i) {
    // compare with exsit paths
    bool new_path = true;

    for (int j = 0; j < exist_paths_id.size(); ++j) {
      // compare with one path
      bool same_topo = sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0);

      if (same_topo) {
        new_path = false;
        break;
      }
    }

    if (new_path) {
      exist_paths_id.push_back(i);
    }
  }

  // save pruned paths
  for (int i = 0; i < exist_paths_id.size(); ++i) {
    pruned_paths.push_back(paths[exist_paths_id[i]]);
  }

  std::cout << ", pruned path num: " << pruned_paths.size();

  return pruned_paths;
}

vector<vector<Eigen::Vector3d>> TopologyPRM::selectShortPaths(vector<vector<Eigen::Vector3d>>& paths,
                                                              int step) {
  /* ---------- only reserve top short path ---------- */
  vector<vector<Eigen::Vector3d>> short_paths;
  vector<Eigen::Vector3d> short_path;
  double min_len;

  for (int i = 0; i < reserve_num_ && paths.size() > 0; ++i) {
    int path_id = shortestPath(paths);
    if (i == 0) {
      short_paths.push_back(paths[path_id]);
      min_len = pathLength(paths[path_id]);
      paths.erase(paths.begin() + path_id);
    } else {
      double rat = pathLength(paths[path_id]) / min_len;
      if (rat < ratio_to_short_) {
        short_paths.push_back(paths[path_id]);
        paths.erase(paths.begin() + path_id);
      } else {
        break;
      }
    }
  }
  std::cout << ", select path num: " << short_paths.size();

  /* ---------- merge with start and end segment ---------- */
  for (int i = 0; i < short_paths.size(); ++i) {
    short_paths[i].insert(short_paths[i].begin(), start_pts_.begin(), start_pts_.end());
    short_paths[i].insert(short_paths[i].end(), end_pts_.begin(), end_pts_.end());
  }
  for (int i = 0; i < short_paths.size(); ++i) {
    shortcutPath(short_paths[i], i, 5);
    short_paths[i] = short_paths_[i];
  }

  short_paths = pruneEquivalent(short_paths);

  return short_paths;
}

bool TopologyPRM::sameTopoPath(const vector<Eigen::Vector3d>& path1,
                               const vector<Eigen::Vector3d>& path2, double thresh) {
  // calc the length
  double len1 = pathLength(path1);
  double len2 = pathLength(path2);

  double max_len = max(len1, len2);

  int pt_num = ceil(max_len / resolution_);

  // std::cout << "pt num: " << pt_num << std::endl;

  vector<Eigen::Vector3d> pts1 = discretizePath(path1, pt_num);
  vector<Eigen::Vector3d> pts2 = discretizePath(path2, pt_num);

  Eigen::Vector3d pc;
  for (int i = 0; i < pt_num; ++i) {
    if (!lineVisib(pts1[i], pts2[i], thresh, pc)) {
      return false;
    }
  }

  return true;
}

int TopologyPRM::shortestPath(vector<vector<Eigen::Vector3d>>& paths) {
  int short_id = -1;
  double min_len = 100000000;
  for (int i = 0; i < paths.size(); ++i) {
    double len = pathLength(paths[i]);
    if (len < min_len) {
      short_id = i;
      min_len = len;
    }
  }
  return short_id;
}
double TopologyPRM::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;

  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

vector<Eigen::Vector3d> TopologyPRM::discretizePath(const vector<Eigen::Vector3d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);

  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list[i]);
  }

  // calc pt_num points along the path
  double len_total = len_list.back();
  double dl = len_total / double(pt_num - 1);
  double cur_l;

  vector<Eigen::Vector3d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;

    // find the range cur_l in
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // find lambda and interpolate
    double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

vector<Eigen::Vector3d> TopologyPRM::pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num) {
  return discretizePath(path, pt_num);
}

void TopologyPRM::shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num) {
  vector<Eigen::Vector3d> short_path = path;
  vector<Eigen::Vector3d> last_path;

  for (int k = 0; k < iter_num; ++k) {
    last_path = short_path;

    vector<Eigen::Vector3d> dis_path = discretizePath(short_path);

    if (dis_path.size() < 2) {
      short_paths_[path_id] = dis_path;
      return;
    }

    /* visibility path shortening */
    Eigen::Vector3d colli_pt, grad, dir, push_dir;
    double dist;
    short_path.clear();
    short_path.push_back(dis_path.front());
    for (int i = 1; i < dis_path.size(); ++i) {
      if (lineVisib(short_path.back(), dis_path[i], resolution_, colli_pt, path_id)) continue;

      edt_environment_->evaluateEDTWithGrad(colli_pt, -1, dist, grad);
      if (grad.norm() > 1e-3) {
        grad.normalize();
        dir = (dis_path[i] - short_path.back()).normalized();
        push_dir = grad - grad.dot(dir) * dir;
        push_dir.normalize();
        colli_pt = colli_pt + resolution_ * push_dir;
      }
      short_path.push_back(colli_pt);
    }
    short_path.push_back(dis_path.back());

    /* break if no shortcut */
    double len1 = pathLength(last_path);
    double len2 = pathLength(short_path);
    if (len2 > len1) {
      // ROS_WARN("pause shortcut, l1: %lf, l2: %lf, iter: %d", len1, len2, k +
      // 1);
      short_path = last_path;
      break;
    }
  }

  short_paths_[path_id] = short_path;
}

void TopologyPRM::shortcutPaths() {
  short_paths_.resize(raw_paths_.size());

  if (parallel_shortcut_) {
    vector<thread> short_threads;
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads.push_back(thread(&TopologyPRM::shortcutPath, this, raw_paths_[i], i, 1));
    }
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads[i].join();
    }
  } else {
    for (int i = 0; i < raw_paths_.size(); ++i) shortcutPath(raw_paths_[i], i);
  }
}

vector<Eigen::Vector3d> TopologyPRM::discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  Eigen::Vector3d dir = p2 - p1;
  double len = dir.norm();
  int seg_num = ceil(len / resolution_);

  vector<Eigen::Vector3d> line_pts;
  if (seg_num <= 0) {
    return line_pts;
  }

  for (int i = 0; i <= seg_num; ++i) line_pts.push_back(p1 + dir * double(i) / double(seg_num));

  return line_pts;
}

vector<Eigen::Vector3d> TopologyPRM::discretizePath(vector<Eigen::Vector3d> path) {
  vector<Eigen::Vector3d> dis_path, segment;

  if (path.size() < 2) {
    ROS_ERROR("what path? ");
    return dis_path;
  }

  for (int i = 0; i < path.size() - 1; ++i) {
    segment = discretizeLine(path[i], path[i + 1]);

    if (segment.size() < 1) continue;

    dis_path.insert(dis_path.end(), segment.begin(), segment.end());
    if (i != path.size() - 2) dis_path.pop_back();
  }
  return dis_path;
}

vector<vector<Eigen::Vector3d>> TopologyPRM::discretizePaths(vector<vector<Eigen::Vector3d>>& path) {
  vector<vector<Eigen::Vector3d>> dis_paths;
  vector<Eigen::Vector3d> dis_path;

  for (int i = 0; i < path.size(); ++i) {
    dis_path = discretizePath(path[i]);

    if (dis_path.size() > 0) dis_paths.push_back(dis_path);
  }

  return dis_paths;
}

Eigen::Vector3d TopologyPRM::getOrthoPoint(const vector<Eigen::Vector3d>& path) {
  Eigen::Vector3d x1 = path.front();
  Eigen::Vector3d x2 = path.back();

  Eigen::Vector3d dir = (x2 - x1).normalized();
  Eigen::Vector3d mid = 0.5 * (x1 + x2);

  double min_cos = 1000.0;
  Eigen::Vector3d pdir;
  Eigen::Vector3d ortho_pt;

  for (int i = 1; i < path.size() - 1; ++i) {
    pdir = (path[i] - mid).normalized();
    double cos = fabs(pdir.dot(dir));

    if (cos < min_cos) {
      min_cos = cos;
      ortho_pt = path[i];
    }
  }

  return ortho_pt;
}

// search for useful path in the topo graph by DFS
vector<vector<Eigen::Vector3d>> TopologyPRM::searchPaths() {
  raw_paths_.clear();

  vector<GraphNode::Ptr> visited;
  visited.push_back(graph_.front());

  depthFirstSearch(visited);

  // sort the path by node number
  int min_node_num = 100000, max_node_num = 1;
  vector<vector<int>> path_list(100);
  for (int i = 0; i < raw_paths_.size(); ++i) {
    if (int(raw_paths_[i].size()) > max_node_num) max_node_num = raw_paths_[i].size();
    if (int(raw_paths_[i].size()) < min_node_num) min_node_num = raw_paths_[i].size();
    path_list[int(raw_paths_[i].size())].push_back(i);
  }

  // select paths with less nodes
  vector<vector<Eigen::Vector3d>> filter_raw_paths;
  for (int i = min_node_num; i <= max_node_num; ++i) {
    bool reach_max = false;
    for (int j = 0; j < path_list[i].size(); ++j) {
      filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
      if (filter_raw_paths.size() >= max_raw_path2_) {
        reach_max = true;
        break;
      }
    }
    if (reach_max) break;
  }
  std::cout << ", raw path num: " << raw_paths_.size() << ", " << filter_raw_paths.size();

  raw_paths_ = filter_raw_paths;

  return raw_paths_;
}

void TopologyPRM::depthFirstSearch(vector<GraphNode::Ptr>& vis) {
  GraphNode::Ptr cur = vis.back();

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // check reach goal
    if (cur->neighbors_[i]->id_ == 1) {
      // add this path to paths set
      vector<Eigen::Vector3d> path;
      for (int j = 0; j < vis.size(); ++j) {
        path.push_back(vis[j]->pos_);
      }
      path.push_back(cur->neighbors_[i]->pos_);

      raw_paths_.push_back(path);
      if (raw_paths_.size() >= max_raw_path_) return;

      break;
    }
  }

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // skip reach goal
    if (cur->neighbors_[i]->id_ == 1) continue;

    // skip already visited node
    bool revisit = false;
    for (int j = 0; j < vis.size(); ++j) {
      if (cur->neighbors_[i]->id_ == vis[j]->id_) {
        revisit = true;
        break;
      }
    }
    if (revisit) continue;

    // recursive search
    vis.push_back(cur->neighbors_[i]);
    depthFirstSearch(vis);
    if (raw_paths_.size() >= max_raw_path_) return;

    vis.pop_back();
  }
}

void TopologyPRM::setEnvironment(const EDTEnvironment::Ptr& env) { this->edt_environment_ = env; }

bool TopologyPRM::triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2) {
  // get the traversing points along p1-p2
  vector<Eigen::Vector3d> pts;

  Eigen::Vector3d dir = p2 - p1;
  double length = dir.norm();
  int seg_num = ceil(length / resolution_);

  Eigen::Vector3d pt1;
  for (int i = 1; i < seg_num; ++i) {
    pt1 = p1 + dir * double(i) / double(seg_num);
    pts.push_back(pt1);
  }

  // test visibility
  for (int i = 0; i < pts.size(); ++i) {
    {
      return false;
    }
  }

  return true;
}
// TopologyPRM::
}  // namespace fast_planner