
#include "../convex_hull_lib/point_hull.hh"

#include "gtest_wrapper.hpp"


#include <filesystem>
#include <string>

namespace fs = std::filesystem;

std::string get_test_input_directory() {
  std::string test_case =
      ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
  std::string test_name =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
  fs::path path(INPUT_DATA_DIR);
  path /= test_case;
  path /= test_name;
  if (!fs::exists(path))
    return std::string();
  return path.generic_string();
}

std::string get_test_output_directory() {
  std::string test_case =
      ::testing::UnitTest::GetInstance()->current_test_info()->test_case_name();
  std::string test_name =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
  fs::path path(OUTPUT_DATA_DIR);
  if (!fs::exists(path))
    fs::create_directory(path);
  path /= test_case;
  if (!fs::exists(path))
    fs::create_directory(path);
  path /= test_name;
  if (!fs::exists(path)) {
    fs::create_directory(path);
    fs::path data_dir(get_test_input_directory());
    if (fs::exists(data_dir)) {
      std::error_code ec;
      fs::create_directory_symlink(data_dir, path / "data_link.lnk", ec);
      std::cout << ec.message();
    }
  }
  return path.generic_string();
}

std::string set_test_output_directory_as_current() {
  auto out_dir = get_test_output_directory();
  fs::current_path(out_dir);
  return out_dir;
}

TEST(CvxHull, Basic00) {
  set_test_output_directory_as_current();
  Points pts{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 1},
             {1, 0, 0}, {1, 0, 1}, {1, 1, 0}, {1, 1, 1}};
  auto mesh = make_convex_hull(pts);
  mesh->compact();
  save_mesh(mesh);
}

TEST(CvxHull, Basic01) {
  set_test_output_directory_as_current();
  Points pts{{0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0},
             {0, 0, 1}, {2, 0, 1}, {2, 1, 1}, {1, 1, 1}, {1, 2, 1}, {0, 2, 1}};
  auto mesh = make_convex_hull(pts);
  mesh->compact();
  save_mesh(mesh);
}
