set(output_lib "convex_hull_test")
include_directories("${gtest_SOURCE_DIR}/include")

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${output_lib} ${SRC_FILES} ${HEADERS})
target_link_libraries(${output_lib} PRIVATE convex_hulll_lib PRIVATE gtest)

target_compile_options(${output_lib}
    PRIVATE "-DINPUT_DATA_DIR=\"${CMAKE_CURRENT_LIST_DIR}/data/\""
    PRIVATE "-DOUTPUT_DATA_DIR=\"${PROJECT_BINARY_DIR}/test_artifacts/\"")

# Adds tests to CTest from GoogleTest.
if (MSVC) # On linux ctest with sanitizer does not work
    include(GoogleTest)
    gtest_discover_tests(${output_lib})
endif (MSVC)
