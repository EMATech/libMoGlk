include_directories(${MyProject_SOURCE_DIR}/src)
link_directories(${MyProject_SOURCE_DIR}/src)
add_executable(Pixel pixel.cc)
target_link_libraries(Pixel Moglk)