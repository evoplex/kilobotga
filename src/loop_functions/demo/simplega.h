/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#ifndef SIMPLE_GA_H
#define SIMPLE_GA_H

#include "controllers/demo/demo_ctrl.h"
#include "loop_functions/demo/demo_lf.h"

#include <QString>

class SimpleGA
{

public:
    SimpleGA(std::vector<DemoCtrl*> &ctrls, TConfigurationNode &t_node);

    void prepareNextGen();
    void loadNextGen();
    void flushIndividuals(const QString &relativePath, const uint32_t curGeneration) const;
    float getGlobalPerformance() const;

private:
    typedef std::vector<LUTMotor> Population;

    std::vector<DemoCtrl*> m_controllers;
    CRandom::CRNG* m_pcRNG;

    size_t m_iPopSize;
    size_t m_iTournamentSize;
    float m_fMutationRate;
    float m_fCrossoverRate;

    Population m_nextGen;

    uint32_t getBestRobotId();
    uint32_t tournamentSelection();
};

#endif // SIMPLE_GA_H
