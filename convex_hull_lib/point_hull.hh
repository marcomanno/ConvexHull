#pragma once

#include "range.hh"
#include "vector.hh"

#include <vector>

struct MeshVertex {
  Geo::VectorD3 m_pt;
  std::vector<size_t> m_adj_idx;
  bool m_boundary = false;
  bool m_to_del = false;
};

struct Mesh {
  Geo::Range<3> m_box;
  std::vector<MeshVertex> m_vert_conn;
  void save(const char* _flnm);
  void compact();
};

using Points = std::vector<Geo::VectorD3>;

Mesh *make_convex_hull(Points &_points);