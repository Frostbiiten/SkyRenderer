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

if (${PLATFORM} STREQUAL "Web")
    set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "Use external fmt")
    set(SPDLOG_DISABLE_DEFAULT_LOGGER ON CACHE BOOL "No default logger")
    set(SPDLOG_NO_EXCEPTIONS ON CACHE BOOL "Disable exceptions for wasm")
    set(SPDLOG_ENABLE_SYSLOG OFF CACHE BOOL "Disable syslog for wasm")
    set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "")
    set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "")
    set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "")
    set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "")
    set(SPDLOG_WCHAR_SUPPORT OFF CACHE BOOL "")
    set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "")
endif()