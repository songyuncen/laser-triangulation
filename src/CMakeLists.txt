# utils library
add_library(utils STATIC 
  extract_laser_line.cc
  back_projection.cc
  laser_triangulation.h
  write_ply.cc 
  fit_3d_plane.cc
  reconstruction.cc
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

macro(add_dev_tool name folder sources)
  add_executable(${name} ${sources})
  target_include_directories(${name}
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(${name}
    PRIVATE
      ${OpenCV_LIBS}
      utils
  )

  if(NOT ${folder} STREQUAL "")
    set_property(TARGET ${name} PROPERTY FOLDER "${folder}")
  endif()

  if(MSVC)
  set_target_properties(${name} 
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
  endif()
endmacro()

add_dev_tool(calibrate_camera Dev calibrate_camera.cc)
add_dev_tool(extract Dev extract.cc)
add_dev_tool(calibrate_laser Dev calibrate_laser.cc)
add_dev_tool(calibrate_movement Dev calibrate_movement.cc)
add_dev_tool(scan Dev scan.cc)
add_dev_tool(demo "" calibrate_sheet_of_light_calplate.cc)

