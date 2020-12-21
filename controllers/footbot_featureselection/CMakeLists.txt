add_library(footbot_featureselection SHARED footbot_featureselection.h footbot_featureselection.cpp nr3.h svd.h)
target_link_libraries(footbot_featureselection
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
