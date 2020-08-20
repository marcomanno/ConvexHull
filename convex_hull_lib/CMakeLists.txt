set(output_lib "convex_hulll_lib")

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.hh ${CMAKE_CURRENT_SOURCE_DIR}/*.inl)
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

source_group("Header Files\\SDK" FILES ${SDK_HEADERS})
source_group("Header Files\\inc" FILES ${HEADERS})

add_library(${output_lib} STATIC ${SRC_FILES} ${HEADERS})

set_target_properties(${output_lib} PROPERTIES CMAKE_<CONFIG>_POSTFIX "")

target_include_directories(${output_lib} 
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
