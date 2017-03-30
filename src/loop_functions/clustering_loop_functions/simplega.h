/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#ifndef SIMPLE_GA_H
#define SIMPLE_GA_H

#include <controllers/kilobot_clustering/kilobot_clustering.h>
#include <loop_functions/clustering_loop_functions/clustering_loop_functions.h>

#include <QString>

class CSimpleGA
{

public:
    CSimpleGA(std::vector<CKilobotClustering *> &ctrls, TConfigurationNode &t_node);

    void prepareNextGen();
    void loadNextGen();
    void flushIndividuals(const QString &relativePath, const uint32_t curGeneration) const;
    float getGlobalPerformance() const;

private:
    typedef std::vector<LUTMotor> Population;

    std::vector<CKilobotClustering*> m_controllers;
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
