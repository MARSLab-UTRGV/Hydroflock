include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(hydroflock_loop_functions MODULE 
  hydroflock_qt_user_functions.h
  hydroflock_qt_user_functions.cpp)

target_link_libraries(hydroflock_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})