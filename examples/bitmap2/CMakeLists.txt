include_directories(${MyProject_SOURCE_DIR}/src)
link_directories(${MyProject_SOURCE_DIR}/src)
add_executable(Bitmap2 bitmap.cc)
target_link_libraries(Bitmap2 Moglk)