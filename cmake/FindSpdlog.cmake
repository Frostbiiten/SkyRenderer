# spdlog
find_package(spdlog QUIET)
if (NOT spdlog_FOUND)
    include(FetchContent)
    FetchContent_Declare(
            spdlog
            GIT_REPOSITORY https://github.com/gabime/spdlog.git
            GIT_TAG v1.14.1
            GIT_SHALLOW 1
    )
    FetchContent_MakeAvailable(spdlog)
endif()