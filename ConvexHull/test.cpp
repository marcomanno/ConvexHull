#include "pch.h"

#include "../convex_hulll_lib/CPointHull.h"

#include <string>

static std::string folder = "C:/Users/amaglim/source/repos/ConvexHull/test_result/";

void save_mesh(Mesh *m) {
  auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string fname =
      folder + test_info->test_case_name() + '.' + test_info->name() + ".obj";
  m->save(fname.c_str());
}

TEST(TestCaseName, TestName) {
  Points pts{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 1},
             {1, 0, 0}, {1, 0, 1}, {1, 1, 0}, {1, 1, 1}};
  auto mesh = make_convex_hull(pts);
  mesh->compact();
  save_mesh(mesh);
}