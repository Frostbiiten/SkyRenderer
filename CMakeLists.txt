cmake_minimum_required(VERSION 3.26)

project(RaylibTest)
set(CMAKE_CXX_STANDARD 20)

# Dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

include(cmake/FindFmt.cmake)
include(cmake/FindSpdlog.cmake)
include(cmake/FindPhysFS.cmake)
include(cmake/FindRaylib.cmake)
include(cmake/FindSIMDE.cmake)

# Define PROJECT_SOURCES as a list of all source files (src)
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.h"
)

# Define PROJECT_INCLUDE to be the path to the include directory of the project
set(PROJECT_INCLUDE "${CMAKE_CURRENT_LIST_DIR}/src/")

set(SOURCES main.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})
target_sources(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_INCLUDE})

# link libs
target_link_libraries(${PROJECT_NAME} PRIVATE raylib raylib_cpp) # raylib + wrapper
target_link_libraries(${PROJECT_NAME} PRIVATE fmt spdlog) # fmt + spdlog
target_link_libraries(${PROJECT_NAME} PRIVATE physfs-static) # physfs
target_include_directories(${PROJECT_NAME} PRIVATE ${simde_SOURCE_DIR}) #simde

if (APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework IOKit")
    target_link_libraries(${PROJECT_NAME} "-framework Cocoa")
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL")
endif()

# Web Configurations
if (${PLATFORM} STREQUAL "Web")
    # Tell Emscripten to build an example.html file.
    set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".html")

    # Required linker flags for using Raylib with Emscripten
    target_link_options(${PROJECT_NAME} PRIVATE -sEXPORTED_FUNCTIONS=['_main','_malloc'] -sEXPORTED_RUNTIME_METHODS=ccall -sUSE_GLFW=3)
endif()