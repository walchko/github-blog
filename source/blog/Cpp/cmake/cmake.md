# CMake

Here are some commonly use `cmake` things I use:

- `PROJECT_SOURCE_DIR`: gives access to top level source directory
- `$ENV{HOME}`: grab environmental variables like `${HOME}` path
- `include_directories(path)`: manually add include path
- `link_directories(path)`: manually add link path
- `set(VARIABLE value)`: set a variable
- `list(APPEND VAR_NAME "${STUFF}")`: create a list
- `message(STATUS "my message ${some_variable}")`: print a message

# Examples

```cmake
cmake_minimum_required(VERSION 3.10)
PROJECT("test")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "-------------------------------------")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
message(STATUS "-------------------------------------")

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(${PROJECT_NAME} test.cpp)
```

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
