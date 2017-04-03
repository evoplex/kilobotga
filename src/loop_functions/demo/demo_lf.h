/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#ifndef DEMO_LOOP_FUNCTIONS_H
#define DEMO_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>

#include "controllers/demo/demo_ctrl.h"
#include "simplega.h"

#include <QString>

class SimpleGA;

/**
 * @brief The DemoLF class
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class DemoLF : public CLoopFunctions {

public:
    DemoLF();
    virtual ~DemoLF() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void Reset();
    virtual void PostExperiment();

private:
    /**
     * Simulation mode.
     * 0 : Run a new experiment
     * 1 : Reproduce an experiment (read from files)
     * 2 : Testing settings (single run)
     */
    enum SIMULATION_MODE {
        NEW_EXPERIMENT,
        READ_EXPERIMENT,
        TEST_SETTINGS
    };

    std::vector<CKilobotEntity*> m_entities;
    std::vector<DemoCtrl*> m_controllers;
    SimpleGA* m_pGA;
    CRandom::CRNG* m_pcRNG;

    SIMULATION_MODE m_eSimMode;
    uint32_t m_iCurGeneration;
    QString m_sRelativePath;

    // stuff loaded from the xml script
    size_t m_iPopSize;
    size_t m_iMaxGenerations;
    CRange<Real> m_arenaSideX;
    CRange<Real> m_arenaSideY;

    void loadLUTMotor(const uint32_t kbId, const QString& absoluteFilePath);
    void loadExperiment();
};

#endif
