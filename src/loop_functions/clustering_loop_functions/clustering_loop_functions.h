/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#ifndef CLUSTERING_LOOP_FUNCTIONS_H
#define CLUSTERING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <controllers/kilobot_clustering/kilobot_clustering.h>

#include "simplega.h"

#include <QString>

class CSimpleGA;

/**
 * @brief The CClusteringLoopFunctions class
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class CClusteringLoopFunctions : public CLoopFunctions {

public:
    CClusteringLoopFunctions();
    virtual ~CClusteringLoopFunctions() {}

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
    std::vector<CKilobotClustering*> m_controllers;
    CSimpleGA* m_cGA;
    CRandom::CRNG* m_pcRNG;

    SIMULATION_MODE m_eSimMode;
    int m_iCurGeneration;
    QString m_sRelativePath;

    // stuff loaded from the xml script
    int m_iPopSize;
    int m_iMaxGenerations;
    CRange<Real> m_arenaSideX;
    CRange<Real> m_arenaSideY;

    bool loadLUTMotor(int kbId, QString absoluteFilePath);
    bool robotExists(const uint32_t id);
    void loadExperiment();
};

#endif
