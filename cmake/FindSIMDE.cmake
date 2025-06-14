include(FetchContent)
FetchContent_Declare(
        simde
        GIT_REPOSITORY https://github.com/simd-everywhere/simde.git
        GIT_TAG        v0.8.2
        GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(simde)