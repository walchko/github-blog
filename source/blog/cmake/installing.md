---
title: Installing CMake Projects
date: 27 Nov 2019
---

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