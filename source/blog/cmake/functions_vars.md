---
title: Cmake Functions and Variables
date: 27 Nov 2019
---

Here are some commonly use `cmake` things I use:

- Useful variable: `${VAR}`
    - `PROJECT_SOURCE_DIR`: gives access to top level source directory
    - `CMAKE_CURRENT_SOURCE_DIR`: gives access to the current directory
- `$ENV{HOME}`: grab environmental variables like `${HOME}` path
- `project(name VERSION 1.2.3 LANGUAGES CXX) 
    - `PROJECT_NAME` set by above
- Global:
    - `include_directories(path)`: manually add include path
    - `link_directories(path)`: manually add link path
- Targets:
    - `target_link_libraries(TARGET PUBLIC|PRIVATE dirs)`: link libraries per target
    - `target_include_directories(TARGET PUBLIC|PRIVATE libs)`: set include path per target
- `set(VARIABLE value)`: set a variable to a value
- `list(APPEND VAR_NAME "${STUFF}")`: create a list of stuff
- `message(STATUS "my message ${some_variable}")`: print a message when cmake runs
