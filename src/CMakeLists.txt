# calibrate camera
add_executable(calibrate_camera calibrate_camera.cc)
target_include_directories(calibrate_camera
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(calibrate_camera
  PRIVATE
    ${OpenCV_LIBS}
)

