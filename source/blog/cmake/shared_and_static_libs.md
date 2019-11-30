---
title: Building Both Shared and Static Libraries
date: 27 Nov 2019
---

The idea here is build your library *once* but only to the object
level. Then use that object to build either (or both) a static
or shared library

```cmake
add_library(${PROJECT_NAME} SHARED $<TARGET_OBJECTS:obj>)
add_library(${PROJECT_NAME}_static STATIC $<TARGET_OBJECTS:obj>)

add_library(obj OBJECT
    src/ascii.cpp
    src/bsocket.cpp
    src/mcsocket.cpp
    src/ssocket.cpp
)

target_include_directories(obj
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${PROJECT_BINARY_DIR}/include
)
```
