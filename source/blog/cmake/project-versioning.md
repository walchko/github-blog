---
title: Setting Project Versions
date: 27 Nov 2019
image: "https://i.pinimg.com/564x/cc/fb/ed/ccfbed6161acb1a662e3930f090848e6.jpg"
image-height: "300px"
---

```cmake
cmake_minimum_required(VERSION 3.10)
PROJECT("test" VERSION 1.2.3 LANGUAGES CXX)

set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
```
