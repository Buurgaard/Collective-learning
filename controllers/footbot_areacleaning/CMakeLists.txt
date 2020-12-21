add_library(footbot_areacleaning SHARED footbot_areacleaning.h footbot_areacleaning.cpp)
target_link_libraries(footbot_areacleaning
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
