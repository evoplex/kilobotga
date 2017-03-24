add_library(kilobot_clustering SHARED kilobot_clustering.h kilobot_clustering.cpp)

target_link_libraries(kilobot_clustering
    argos3core_simulator
    argos3plugin_simulator_kilobot
    argos3plugin_simulator_genericrobot
    #argos3plugin_simulator_kilolib
)
