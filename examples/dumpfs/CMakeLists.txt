include_directories(${MyProject_SOURCE_DIR}/src)
link_directories(${MyProject_SOURCE_DIR}/src)
add_executable(Dumpfs dumpfs.cc)
target_link_libraries(Dumpfs Moglk)