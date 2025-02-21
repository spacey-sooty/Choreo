# Disable in-source builds to prevent source tree corruption
if("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
    message(
        FATAL_ERROR
        "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
"
    )
endif()

cmake_minimum_required(VERSION 3.22)
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules"
)

# Set default build type to release with debug info (i.e. release mode
# optimizations are performed, but debug info still exists).
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "" FORCE)
endif()

include(FetchContent)
fetchcontent_declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir
    # main on 2025-01-14
    GIT_TAG 742092895f076733f5f9c584e4aae71d86ec5ea1
    PATCH_COMMAND
        git apply ${CMAKE_CURRENT_SOURCE_DIR}/cmake/0001-Downgrade-to-C-20.patch
    UPDATE_DISCONNECTED 1
)
fetchcontent_makeavailable(Sleipnir)

add_subdirectory(trajoptlib)
