add_library(demo_ctrl SHARED demo_ctrl.h demo_ctrl.cpp)

target_link_libraries(demo_ctrl
    argos3core_simulator
    argos3plugin_simulator_kilobot
    argos3plugin_simulator_genericrobot
)
