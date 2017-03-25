add_executable(clustering main.cpp)

target_link_libraries(clustering
    kilobot_clustering
    argos3core_simulator
    clustering_loop_functions
)
