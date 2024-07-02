# Find and include the GLUT and OpenGL libraries
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

# Add the library for loop functions
add_library(hydroflock_loop_functions MODULE 
  hydroflock_qt_user_functions.h
  hydroflock_qt_user_functions.cpp)

# Link against ARGoS and OpenGL libraries
target_link_libraries(hydroflock_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES} 
  ${OPENGL_LIBRARIES} 
  ${GLUT_LIBRARIES})

# Ensure the shared library is placed in the correct output directory
set_target_properties(hydroflock_loop_functions PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/loop_functions)
