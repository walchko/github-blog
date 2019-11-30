---
title: CMake
date: 15 Jun 2019
---


# Examples

```cmake
cmake_minimum_required(VERSION 3.10)
project(test)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB SOURCES *.cpp)

message(STATUS "Tests ----------------------")
foreach(src ${SOURCES})
    get_filename_component(name ${src} NAME_WE)
    message(STATUS " -> ${name}")
    add_executable(${name} ${src})
    # target_link_libraries(${name} some_library)
endforeach()
```

```cmake
# turn this on with cmake -DBUILD_MSGPACK=ON
option(BUILD_MSGPACK "Build the message pack messages" ON)
```

## Library

```
add_library(math SHARED lib.cpp)
add_library(math STATIC lib.cpp)
```

Shared Library File Extensions:

- Windows: `.dll`
- Mac OS X: `.dylib`
- Linux: `.so`

Static Library File Extensions:

- Windows: `.lib`
- Mac OS X: `.a`
- Linux: `.a`

Run script during install

```cmake
install(CODE "execute_process(COMMAND my_script.sh)")
```

# References

- [cmake install docs](https://cmake.org/cmake/help/v3.0/command/install.html)
- [cmake tutorial](https://medium.com/@onur.dundar1/cmake-tutorial-585dd180109b)
- [github cmake examples](https://github.com/ttroy50/cmake-examples)
- [select cmake articles](https://github.com/onqtam/awesome-cmake)
- [github cmake templates](https://github.com/acdemiralp/cmake_templates)
- [A really good website for CMake](https://cliutils.gitlab.io/modern-cmake/)
