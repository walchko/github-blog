---
title: CMake
date: 15 Jun 2019
---

Here are some commonly use `cmake` things I use:

- Useful variable: `${VAR}`
    - `PROJECT_SOURCE_DIR`: gives access to top level source directory
    - `CMAKE_CURRENT_SOURCE_DIR`: gives access to the current directory
- `$ENV{HOME}`: grab environmental variables like `${HOME}` path
- `project(name VERSION 1.2.3 LANGUAGES CXX) 
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

cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
# cmake_host_system_information(RESULT CORES QUERY NUMBER_OF_LOGICAL_CORES)
cmake_host_system_information(RESULT OSN QUERY OS_NAME)
cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

message(STATUS "-------------------------------------")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
message(STATUS "-------------------------------------")
message(STATUS " ${HOST}")
message(STATUS " ${OSN}: ${OS_VERSION}")
message(STATUS " ${PROC}")
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
## Packaging and Version

```cmake
cmake_minimum_required(VERSION 3.10)
PROJECT("test" VERSION 1.2.3 LANGUAGES CXX)

set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
include(CPack)
```

## Install

```cmake
set(BIN_DESTINATION /my_dir/bin)
# set other paths

install(PROGRAMS
  scripts/my_python_script
  DESTINATION ${BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${LIB_DESTINATION}  # static libraries
  LIBRARY DESTINATION ${LIB_DESTINATION}  # shared libraries
  RUNTIME DESTINATION ${BIN_DESTINATION}  # executables
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".git" EXCLUDE
)

install(FILES
  myfile1.json
  myfile2.yaml
  DESTINATION ${SHARE_DESTINATION}
)
```

Run script during install

```cmake
install(CODE "execute_process(COMMAND my_script.sh)")
```

# References

- [cmake install docs](https://cmake.org/cmake/help/v3.0/command/install.html)
