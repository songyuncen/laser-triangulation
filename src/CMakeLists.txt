# utils
add_library(utils STATIC 
  extract_laser_line.cc
  laser_triangulation.h
)

target_include_directories(utils
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(utils
  PRIVATE
  ${OpenCV_LIBS}
)

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

# extract
add_executable(extract extract.cc)
target_include_directories(extract
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(extract
  PRIVATE
    ${OpenCV_LIBS}
    utils
)

