add_library(footbot_hydroflock MODULE footbot_hydroflock.h footbot_hydroflock.cpp)
add_library(rab_dsr SHARED rab_dsr.h rab_dsr.cpp)
target_link_libraries(footbot_hydroflock
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  rab_dsr)
