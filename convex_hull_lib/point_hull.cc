#include "point_hull.hh"

#include <algorithm>
#include <fstream>
#include <set>
#include <string>

static const size_t INVALID = std::numeric_limits<size_t>::max();

struct Evaluator {
  Evaluator(const Geo::VectorD3 &_mid_pt0, const Geo::VectorD3 &_mid_pt1,
            size_t _split_coord, double _split_val)
      : m_split_coord(_split_coord), m_split_val(_split_val) {
    m_center = get_center_point(_mid_pt0, _mid_pt1);
  }

  double evaluate(const Geo::VectorD3 &_pt0, const Geo::VectorD3 &_pt1,
                  Geo::VectorD3 &_pt_pl) {
    _pt_pl = get_center_point(_pt0, _pt1);
    return Geo::length_square(_pt_pl - m_center);
  }

  double evaluate_angle(const Geo::VectorD3 &_pt0, const Geo::VectorD3 &_pt1,
                        const Geo::VectorD3 &_ref, Geo::VectorD3 &_pt_pl) {
    _pt_pl = get_center_point(_pt0, _pt1);
    auto pt_to_ref = _pt_pl - _ref;
    auto cen_to_ref = m_center - _ref;
    auto sin_a = (pt_to_ref % cen_to_ref)[m_split_coord];
    if (sin_a < 0) // Check the rotation is jot negative.
      return -1;
    auto cos_a = pt_to_ref * cen_to_ref;
    return atan2(sin_a, cos_a);
  }

  Geo::VectorD3 get_center_point(const Geo::VectorD3 &_pt0,
                                 const Geo::VectorD3 &_pt1) {
    auto center_pt = (_pt1 + _pt0) * .5;
    center_pt[m_split_coord] = 0;
    return center_pt;
  }

  size_t m_split_coord;
  double m_split_val;
  Geo::VectorD3 m_center;
};

static size_t replace_vertex(Mesh *mm, size_t v0, size_t v1, size_t oth,
                             std::array<size_t, 2> &adj) {
  const auto &orig = mm->m_vert_conn[v0].m_pt;
  auto dir = mm->m_vert_conn[v1].m_pt - orig;
  dir /= Geo::length(dir);
  auto find_dir = [mm, &orig, &dir](size_t v) {
    auto res = mm->m_vert_conn[v].m_pt - orig;
    return res - (res * dir) * dir;
  };
  auto adj0 = find_dir(adj[0]);
  auto adj1 = find_dir(adj[1]);
  auto oth_v = find_dir(oth);
  auto angle_ref = fabs(Geo::angle(adj0, adj1));
  auto angle_0 = fabs(Geo::angle(adj0, oth_v));
  auto angle_1 = fabs(Geo::angle(adj1, oth_v));
  size_t disconnect;
  if (angle_0 > angle_1 && angle_0 > angle_ref) {
    disconnect = adj[1];
    adj[1] = oth;
  } else if (angle_1 > angle_0 && angle_1 > angle_ref) {
    disconnect = adj[0];
    adj[0] = oth;
  } else
    disconnect = oth;
  return disconnect;
}

using Link = std::array<size_t, 2>;

static void remove_internal_links(Mesh *m, const std::vector<Link> &new_links) {
  if (new_links.size() < 2)
    return;
  std::vector<Link> links_for_removal;
  auto link_0 = std::prev(new_links.end());
  for (auto link_1 = new_links.begin(); link_1 != new_links.end();
       link_0 = link_1++) {
    size_t mesh_idx;
    if ((*link_0)[0] == (*link_1)[0])
      mesh_idx = 1;
    else if ((*link_0)[1] == (*link_1)[1])
      mesh_idx = 0;
    else
      throw "Error";
    size_t v0 = (*link_0)[mesh_idx];
    size_t v1 = (*link_1)[mesh_idx];
    std::array<size_t, 2> adj;
    size_t cmn_verts = 0;
    for (auto oth0 : m->m_vert_conn[v0].m_adj_idx) {
      for (auto oth1 : m->m_vert_conn[v1].m_adj_idx) {
        if (oth0 == oth1) {
          if (cmn_verts < 2) {
            adj[cmn_verts++] = oth1;
            continue;
          }
          auto disconnect = replace_vertex(m, v0, v1, oth1, adj);
          auto remove_link = [&m](std::array<size_t, 2> link) {
            for (auto i : {0, 1}) {
              auto &v_links = m->m_vert_conn[link[i]].m_adj_idx;
              auto new_end =
                  std::remove(v_links.begin(), v_links.end(), link[1 - i]);
              v_links.erase(new_end, v_links.end());
            }
          };
          auto add_link =
              [&links_for_removal](const Link &link) {
                links_for_removal.push_back(link);
                if (links_for_removal.back()[0] > links_for_removal.back()[1])
                  std::swap(links_for_removal.back()[0],
                            links_for_removal.back()[1]);
              };
          add_link({disconnect, v0});
          add_link({disconnect, v1});
        }
      }
    }
  }
  if (links_for_removal.size() < 2)
    return;
  std::sort(links_for_removal.begin(), links_for_removal.end());
  auto l0 = links_for_removal.begin();
  for (auto l1 = std::next(l0); l1 != links_for_removal.end(); l0 = l1++) {
    if (*l0 != *l1)
      continue;
    for (auto i : {0, 1}) {
      auto &v_links = m->m_vert_conn[(*l1)[i]].m_adj_idx;
      auto new_end = std::remove(v_links.begin(), v_links.end(), (*l1)[1 - i]);
      v_links.erase(new_end, v_links.end());
    }
  }
}

Mesh *merge(std::array<Mesh *, 2> &m, size_t split_coord, double split_val) {
  Link link{};
  Geo::VectorD3 p_mid_plane;
  Evaluator ev(m[0]->m_mid_pt, m[1]->m_mid_pt, split_coord, split_val);
  {
    auto best_val = ev.evaluate(m[0]->m_vert_conn[link[0]].m_pt,
                                m[1]->m_vert_conn[link[1]].m_pt, p_mid_plane);
    bool a_change;
    do {
      a_change = false;
      auto advance = [&best_val, &a_change, &ev, &p_mid_plane](
                         Mesh *_m0, Mesh *_m1, size_t &_i0, size_t _i1) {
        for (auto i0_1 : _m0->m_vert_conn[_i0].m_adj_idx) {
          Geo::VectorD3 p_mid_plane_tmp;
          auto new_val =
              ev.evaluate(_m0->m_vert_conn[i0_1].m_pt,
                          _m1->m_vert_conn[_i1].m_pt, p_mid_plane_tmp);
          if (new_val > best_val) {
            best_val = new_val;
            _i0 = i0_1;
            p_mid_plane = p_mid_plane_tmp;
            a_change = true;
          }
        }
      };
      advance(m[0], m[1], link[0], link[1]);
      advance(m[1], m[0], link[1], link[0]);
    } while (a_change);
  }
  std::vector<Link> new_links;
  new_links.push_back(link);
  Link link_prev{INVALID, INVALID};
  for (;;) {
    double big_angle_new = -1;
    Link link_new = link;
    Geo::VectorD3 p_mid_plane_new;
    auto advance = [&p_mid_plane, &big_angle_new, &ev, &p_mid_plane_new](
                       Mesh *_m0, Mesh *_m1, size_t _i0, size_t _i1,
                       size_t &_i0_new, size_t &_i1_new, const size_t &_used) {
      for (auto i0_1 : _m0->m_vert_conn[_i0].m_adj_idx) {
        if (_used == i0_1)
          continue;
        Geo::VectorD3 p_mid_plane_1;
        auto big_angle_1 = ev.evaluate_angle(_m0->m_vert_conn[i0_1].m_pt,
                                             _m1->m_vert_conn[_i1].m_pt,
                                             p_mid_plane, p_mid_plane_1);
        if (big_angle_1 > big_angle_new) {
          big_angle_new = big_angle_1;
          _i0_new = i0_1;
          _i1_new = _i1;
          p_mid_plane_new = p_mid_plane_1;
        }
      }
    };
    advance(m[0], m[1], link[0], link[1], link_new[0], link_new[1],
            link_prev[0]);
    advance(m[1], m[0], link[1], link[0], link_new[1], link_new[0],
            link_prev[1]);
    if (link == link_new || new_links.front() == link_new)
      break;
    for (auto j : {0, 1}) {
      if (link_new[j] != link[j])
        link_prev[j] = link[j];
    }
    link = link_new;
    new_links.push_back(link);
    p_mid_plane = p_mid_plane_new;
  }
  auto flag_on_boundary = [&new_links, m](bool set_true) {
    for (const auto &inds : new_links) {
      m[1]->m_vert_conn[inds[1]].m_boundary =
          m[0]->m_vert_conn[inds[0]].m_boundary = set_true;
    }
  };
  flag_on_boundary(true);
  std::vector<size_t> vertices_to_remove[2];
  auto dir_centers = m[1]->m_mid_pt - m[0]->m_mid_pt;
  auto link_0 = new_links.begin();
  for (auto link_1 = std::next(link_0); link_1 != new_links.end();
       link_0 = link_1++) {
    size_t mesh_idx;
    if ((*link_0)[0] == (*link_1)[0])
      mesh_idx = 1;
    else if ((*link_0)[1] == (*link_1)[1])
      mesh_idx = 0;
    else
      throw "Error";
    Mesh *mm = m[mesh_idx];
    size_t v0 = (*link_0)[mesh_idx];
    size_t v1 = (*link_1)[mesh_idx];
    std::array<size_t, 2> adj;
    size_t cmn_verts = 0;
    for (auto oth0 : mm->m_vert_conn[v0].m_adj_idx) {
      for (auto oth1 : mm->m_vert_conn[v1].m_adj_idx) {
        if (oth0 == oth1) {
          if (cmn_verts < 2)
            adj[cmn_verts++] = oth1;
          else
            replace_vertex(mm, v0, v1, oth1, adj);
        }
      }
    }
    if (cmn_verts == 2) {
      auto dir_edge = mm->m_vert_conn[v1].m_pt - mm->m_vert_conn[v0].m_pt;
      double vals[2];
      for (size_t j = 0; j < 2; ++j) {
        auto dir_inside =
            mm->m_vert_conn[adj[j]].m_pt - mm->m_vert_conn[v0].m_pt;
        dir_inside -=
            ((dir_inside * dir_edge) / Geo::length_square(dir_edge)) * dir_edge;
        vals[j] = dir_inside * dir_centers;
        if (mesh_idx == 1)
          vals[j] *= -1;
      }
      size_t vertex_to_remove = vals[0] > vals[1] ? adj[0] : adj[1];
      vertices_to_remove[mesh_idx].push_back(vertex_to_remove);
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    auto mm = m[i];
    for (size_t j = 0; j < vertices_to_remove[i].size(); ++j) {
      auto v = vertices_to_remove[i][j];
      if (mm->m_vert_conn[v].m_to_del || mm->m_vert_conn[v].m_boundary)
        continue;
      for (auto v1 : mm->m_vert_conn[v].m_adj_idx) {
        if (mm->m_vert_conn[v1].m_to_del)
          continue;
        if (!mm->m_vert_conn[v1].m_boundary)
          vertices_to_remove[i].push_back(v1);
        else {
          auto it = std::find(mm->m_vert_conn[v1].m_adj_idx.begin(),
                              mm->m_vert_conn[v1].m_adj_idx.end(), v);
          if (it != mm->m_vert_conn[v1].m_adj_idx.end())
            mm->m_vert_conn[v1].m_adj_idx.erase(it);
        }
      }
      mm->m_vert_conn[v].m_adj_idx.clear();
      mm->m_vert_conn[v].m_to_del = true;
    }
  }
  flag_on_boundary(false);
  // Add m[1] to m[0]. Must shift indices
  auto shif_index = m[0]->m_vert_conn.size();
  for (auto &ele : m[1]->m_vert_conn) {
    for (auto &adj : ele.m_adj_idx)
      adj += shif_index;
  }
  m[0]->m_box += m[1]->m_box;
  m[0]->m_mid_pt =
      m[0]->m_mid_pt * static_cast<double>(m[0]->m_vert_conn.size()) +
      m[1]->m_mid_pt * static_cast<double>(m[1]->m_vert_conn.size());
  m[0]->m_vert_conn.insert(m[0]->m_vert_conn.end(), m[1]->m_vert_conn.begin(),
                           m[1]->m_vert_conn.end());
  m[0]->m_mid_pt /= static_cast<double>(m[0]->m_vert_conn.size());
  for (auto &new_link : new_links) {
    new_link[1] += shif_index;
    m[0]->m_vert_conn[new_link[0]].m_adj_idx.push_back(new_link[1]);
    m[0]->m_vert_conn[new_link[1]].m_adj_idx.push_back(new_link[0]);
  }
  // Add the new faces to m[0];
  delete m[1];
  remove_internal_links(m[0], new_links);
  m[0]->compact();
  save_mesh(m[0]);
  return m[0];
}

Mesh *make_convex_hull(Points::iterator _begin, Points::iterator _end) {
  auto size = _end - _begin;
  if (size <= 3) {
    auto m = new Mesh;
    for (auto i = 0; i < size; ++i) {
      auto &el = m->m_vert_conn.emplace_back();
      el.m_pt = *_begin++;
      m->m_box += el.m_pt;
      m->m_mid_pt += el.m_pt;
      for (auto j = size; j-- > 0;) {
        if (j != i)
          el.m_adj_idx.push_back(j);
      }
    }
    m->m_mid_pt /= static_cast<double>(size);
    save_mesh(m);
    return m;
  }
  Geo::Range<3> box;
  for (auto it = _begin; it != _end; ++it) {
    box += *it;
  }
  auto v = box[1] - box[0];
  size_t split_coord = 0;
  double len = 0;
  for (size_t i = 0; i < 3; ++i) {
    auto len_i = std::fabs(v[i]);
    if (len_i > len) {
      len = len_i;
      split_coord = i;
    }
  }
  std::nth_element(
      _begin, _begin + size / 2, _end,
      [split_coord](const Geo::VectorD3 &_a, const Geo::VectorD3 &_b) {
        return _a[split_coord] < _b[split_coord];
      });
  auto mid_iter = _begin + (_end - _begin) / 2;
  auto split_val =
      ((*mid_iter)[split_coord] + (*std::prev(mid_iter))[split_coord]) / 2.;

  std::array<Mesh *, 2> m{make_convex_hull(_begin, mid_iter),
                          make_convex_hull(mid_iter, _end)};
  return merge(m, split_coord, split_val);
}

Mesh *make_convex_hull(Points &_points) {
  return make_convex_hull(_points.begin(), _points.end());
}

void Mesh::compact() {
  auto size = m_vert_conn.size();
  std::vector<size_t> ind_map(size);
  size_t valid_ind = 0;
  std::vector<MeshVertex> new_data;
  for (size_t i = 0; i < size; ++i) {
    if (m_vert_conn[i].m_to_del)
      ind_map[i] = INVALID;
    else {
      new_data.emplace_back(m_vert_conn[i]);
      ind_map[i] = valid_ind++;
    }
  }
  for (auto &v_conn : new_data) {
    for (auto &idx : v_conn.m_adj_idx) {
      idx = ind_map[idx];
    }
  }
  m_vert_conn = std::move(new_data);
}

void common_element(const std::vector<size_t> &_a,
                    const std::vector<size_t> &_b, std::vector<size_t> &_res) {
  _res.clear();
  std::set_intersection(_a.begin(), _a.end(), _b.begin(), _b.end(),
                        std::back_inserter(_res));
}

void Mesh::save(const char *_flnm) {
  compact();
  std::ofstream cc(_flnm);
  for (auto &v_conn : m_vert_conn) {
    cc << "v " << v_conn.m_pt[0] << ' ' << v_conn.m_pt[1] << ' '
       << v_conn.m_pt[2] << std::endl;
    std::sort(v_conn.m_adj_idx.begin(), v_conn.m_adj_idx.end());
  }
  std::array<size_t, 3> ff;
  std::vector<std::array<size_t, 3>> all_faces;
  std::vector<size_t> third_verts;
  for (ff[0] = 0; ff[0] < m_vert_conn.size(); ++ff[0]) {
    for (auto id : m_vert_conn[ff[0]].m_adj_idx) {
      if (id <= ff[0])
        continue;
      ff[1] = id;
      common_element(m_vert_conn[ff[0]].m_adj_idx, m_vert_conn[ff[1]].m_adj_idx,
                     third_verts);
      for (auto id2 : third_verts) {
        if (id2 <= ff[1])
          continue;
        ff[2] = id2;
        std::sort(ff.begin(), ff.end());
        all_faces.push_back(ff);
      }
    }
  }
  std::sort(all_faces.begin(), all_faces.end());
  auto last = std::unique(all_faces.begin(), all_faces.end());
  all_faces.erase(last, all_faces.end());
  for (auto &f : all_faces) {
    cc << "f " << f[0] + 1 << ' ' << f[1] + 1 << ' ' << f[2] + 1 << std::endl;
  }
}

void save_mesh(Mesh *m) {
  static int nn = 0;
  auto prefix = std::to_string(nn++);
  while (prefix.size() < 4)
    prefix = std::string("0") + prefix;
  m->save((prefix + "_mesh.obj").c_str());
}
