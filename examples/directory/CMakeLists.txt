include_directories(${MyProject_SOURCE_DIR}/src)
link_directories(${MyProject_SOURCE_DIR}/src)
add_executable(Directory directory.cc)
target_link_libraries(Directory Moglk)