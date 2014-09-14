include_directories(${MyProject_SOURCE_DIR}/src)
link_directories(${MyProject_SOURCE_DIR}/src)
add_executable(Autodetectbr autodetectbr.cc)
target_link_libraries(Autodetectbr Moglk)