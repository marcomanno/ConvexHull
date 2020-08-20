cmake_minimum_required(VERSION 3.10)
cmake_policy(VERSION 3.10)

if (WIN32)
  set (RUN_PYTHON "py")
else ()
  set (RUN_PYTHON "python3")
endif(WIN32)
set (BUILD_TOOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build_tools")

#Ensure CMAKE_CONFIGURATION_TYPES is set
enable_language(CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_CONFIGURATION_TYPES)
    message("Multi-configuration generator")
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "My multi config types" FORCE)
else()
    message("Single-configuration generator")
endif()

if (UNIX)
    set(CMAKE_INSTALL_RPATH "$ORIGIN/")
    set(CMAKE_BUILD_WITH_INSTALL_RPATH 1)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    set(CMAKE_C_VISIBILITY_PRESET hidden)
    set(LINKER_FLAGS "-Wl,-unresolved-symbols=ignore-in-shared-libs -pthread ")
    set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS})
    set(CMAKE_SHARED_LINKER_FLAGS ${LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS})
    set(CMAKE_CXX_STANDARD_LIBRARIES "${CMAKE_CXX_STANDARD_LIBRARIES} -lstdc++fs") #hack to ensure we link against experimental filesystem
    add_compile_options(-Wall -Werror -pthread)
    if (SANITIZER) 
        add_compile_options("-fsanitize=${SANITIZER}" "-fno-sanitize-recover=all" "-fno-sanitize=vptr")
        link_libraries("-fsanitize=${SANITIZER}") #Linker needs to know about sanitizer to link in runtime. Link libraries can also be used to add link flags.
    endif()
endif()
if (MSVC) #MSVC platform we build only Win64  
    add_compile_options(/WX /W4 /Zi /Zo /MP)
    add_compile_definitions("$<$<CONFIG:DEBUG>:_ITERATOR_DEBUG_LEVEL=1>")
    set(RELEASE_LINKER_FLAGS "/DEBUG /MANIFEST:NO /INCREMENTAL:NO /OPT:REF")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${RELEASE_LINKER_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${RELEASE_LINKER_FLAGS}")
endif (MSVC)
add_compile_definitions("_USE_MATH_DEFINES")

project(ConvexHull)
enable_testing()

#Send all output to fixed location.
set(OUTPUT_DIR_DEBUG "${PROJECT_BINARY_DIR}/lib/debug/${FLAVOUR}")
set(OUTPUT_DIR_RELEASE "${PROJECT_BINARY_DIR}/lib/release/${FLAVOUR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG    ${OUTPUT_DIR_DEBUG})  
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE  ${OUTPUT_DIR_RELEASE})  

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG    ${OUTPUT_DIR_DEBUG})  
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE  ${OUTPUT_DIR_RELEASE})  

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG    ${OUTPUT_DIR_DEBUG})  
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE  ${OUTPUT_DIR_RELEASE})

set(CMAKE_BINARY_DIR ${PROJECT_BINARY_DIR})

### Add Gtest
set(gtest_force_shared_crt ON CACHE BOOL "gtest use shared crt" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "gtest install disabled" FORCE)
set(INSTALL_GMOCK OFF CACHE BOOL "gtest install disabled" FORCE)
add_subdirectory(googletest)

add_subdirectory(convex_hull_lib)
add_subdirectory(convex_hull_test)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT "convex_hull_test")
