# physfs
set(PHYSFS_BUILD_STATIC ON CACHE BOOL "" FORCE)
set(PHYSFS_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(PHYSFS_BUILD_TEST OFF CACHE BOOL "" FORCE)
set(PHYSFS_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(PHYSFS_DISABLE_INSTALL OFF CACHE BOOL "" FORCE)

find_package(PhysFS QUIET)
if (NOT PhysFS_FOUND)
    include(FetchContent)
    FetchContent_Declare(
            PhysFS
            GIT_REPOSITORY https://github.com/icculus/physfs.git
            GIT_TAG release-3.2.0
            GIT_SHALLOW 1
    )
    FetchContent_MakeAvailable(PhysFS)
endif()

# include_directories(${PROJECT_NAME} PUBLIC ${physfs_SOURCE_DIR}/src)
include_directories(${physfs_SOURCE_DIR}/src)
