find_package(Qt5Core)

add_library(kga_loopfunctions SHARED
    abstractga_lf.h
    abstractga_lf.cpp
    demo_lf.h
    demo_lf.cpp
    pd_lf.h
    pd_lf.cpp
)

target_link_libraries(kga_loopfunctions
    kga_controllers
    Qt5::Core
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_media
)
