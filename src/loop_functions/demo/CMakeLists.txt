link_directories(${CMAKE_BINARY_DIR}/controllers/demo_ctrl)

find_package(Qt5Core)

add_library(demo_lf SHARED
    simplega.h
    simplega.cpp
    demo_lf.h
    demo_lf.cpp)

target_link_libraries(demo_lf
    demo_ctrl
    Qt5::Core

    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_kilobot
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media
)
