cmake_minimum_required(VERSION 3.26)

project(RaylibTest)
set(CMAKE_CXX_STANDARD 20)

# Dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

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
target_include_directories(${PROJECT_NAME} PRIVATE ${simde_SOURCE_DIR}) #simde

if (APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework IOKit")
    target_link_libraries(${PROJECT_NAME} "-framework Cocoa")
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL")
endif()

# Web Configurations
if (${PLATFORM} STREQUAL "Web")
    set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".html")

    target_link_options(${PROJECT_NAME} PRIVATE
            "--preload-file=assets"
            "-sEXPORTED_FUNCTIONS=['_main','_malloc']"
            "-sEXPORTED_RUNTIME_METHODS=['ccall']"
            "-sUSE_GLFW=3"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sINITIAL_MEMORY=64MB"
            "-sMAXIMUM_MEMORY=512MB"
            "-sASYNCIFY"
            "-O3"
            "-msimd128"
            "-sWASM_BIGINT=1"
            "--closure=1"
            "-sASSERTIONS=0"
    )

    target_compile_options(${PROJECT_NAME} PRIVATE
            "-O3"
            "-msimd128"
            "-ffast-math"
            "-flto"
    )
endif()