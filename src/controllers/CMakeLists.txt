add_library(kga_controllers SHARED
    abstractga_ctrl.h
    abstractga_ctrl.cpp
    demo_ctrl.h
    demo_ctrl.cpp
)

target_link_libraries(kga_controllers
    argos3core_simulator
    argos3plugin_simulator_kilobot
    argos3plugin_simulator_genericrobot
)
