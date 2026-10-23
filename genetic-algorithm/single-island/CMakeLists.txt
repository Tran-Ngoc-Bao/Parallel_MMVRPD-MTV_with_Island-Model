cmake_minimum_required(VERSION 3.16)
project(ga-single-island CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    CLI11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG        v2.3.2
)
FetchContent_MakeAvailable(CLI11)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

FetchContent_Declare(
    mimalloc
    GIT_REPOSITORY https://github.com/microsoft/mimalloc.git
    GIT_TAG        v2.1.2
    GIT_SHALLOW    TRUE
)
set(MI_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(MI_BUILD_SHARED   OFF CACHE BOOL "" FORCE)
set(MI_BUILD_OBJECT   OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(mimalloc)

add_executable(ga-single-island
    # core (copied from the tabu search, parts the HGS does not use removed)
    src/config.cpp
    src/clusterize.cpp
    src/routes.cpp
    src/solutions.cpp
    # hybrid genetic search
    src/main.cpp
    src/genetics.cpp
    src/crossover.cpp
    src/local_search.cpp
    src/population.cpp
    src/island.cpp
    src/logger.cpp
)

target_include_directories(ga-single-island PRIVATE src)
target_link_libraries(ga-single-island PRIVATE CLI11::CLI11 nlohmann_json::nlohmann_json mimalloc-static)

if(MSVC)
    target_compile_options(ga-single-island PRIVATE /W3 /O2)
else()
    target_compile_options(ga-single-island PRIVATE
        -Wall -Wextra -O3 -march=native -flto -fno-rtti)
    target_link_options(ga-single-island PRIVATE -flto)
endif()
