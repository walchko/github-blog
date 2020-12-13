---
title: Git and CMake
date: 26 Nov 2019
image: "https://i.pinimg.com/564x/86/84/9a/86849aecbb22312ba23cd0506b23dbf4.jpg"
image-height: "300px"
---

Doing `find_package(Git QUIET)` will give you access to:

- `GIT_EXECUTABLE`: Path to Git command-line client.
- `GIT_FOUND`: True if the Git command-line client was found.
- `GIT_VERSION_STRING`: The version of Git found.

## Example

```cmake
message(STATUS "Submodule update")
execute_process(
    COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE GIT_SUBMOD_RESULT
)
if(NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
endif()
```

# References

- [examples](https://cliutils.gitlab.io/modern-cmake/chapters/projects/submodule.html)
