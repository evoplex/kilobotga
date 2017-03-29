link_directories(${CMAKE_BINARY_DIR}/controllers/kilobot_clustering)

find_package(Qt5Core)

add_library(clustering_loop_functions SHARED
    simplega.h
    simplega.cpp
    clustering_loop_functions.h
    clustering_loop_functions.cpp)

target_link_libraries(clustering_loop_functions
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_kilobot
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media
    kilobot_clustering
    Qt5::Core
)
