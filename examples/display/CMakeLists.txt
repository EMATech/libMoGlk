include_directories(${MyProject_SOURCE_DIR}/src)
link_directories(${MyProject_SOURCE_DIR}/src)
add_executable(Display display.cc)
target_link_libraries(Display Moglk)