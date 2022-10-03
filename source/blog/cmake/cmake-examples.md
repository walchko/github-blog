![](https://i.pinimg.com/564x/49/50/85/495085010a8d10d7dd1e2c048c0d34b3.jpg)

# CMake Examples

```cmake
cmake_minimum_required(VERSION 3.14)
project(test)
set(CMAKE_CXX_STANDARD 20)
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

## Simple Library

```cmake
add_library(math SHARED lib.cpp)

# OR

add_library(math STATIC lib.cpp)
```

| OS      | Shared   | Static |
|---------|----------|--------|
| Windows | `.dll`   | `.lib` |
| macOS   | `.dylib` | `.a`   |
| Linux   | `.so`    | `.a`   |

## Static and Shared Library from Same Project

```cmake
# Library =====================================================================
add_library(${PROJECT_NAME} SHARED $<TARGET_OBJECTS:objlib-gecko>)
add_library(${PROJECT_NAME}-static STATIC $<TARGET_OBJECTS:objlib-gecko>)
set_target_properties(${PROJECT_NAME}-static PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# if building STATIC library, an unimportant but annoying error appears about
# no symbols
SET(CMAKE_C_ARCHIVE_CREATE   "<CMAKE_AR> Scr <TARGET> <LINK_FLAGS> <OBJECTS>")
SET(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> Scr <TARGET> <LINK_FLAGS> <OBJECTS>")
SET(CMAKE_C_ARCHIVE_FINISH   "<CMAKE_RANLIB> -no_warning_for_no_symbols -c <TARGET>")
SET(CMAKE_CXX_ARCHIVE_FINISH "<CMAKE_RANLIB> -no_warning_for_no_symbols -c <TARGET>")
```

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
