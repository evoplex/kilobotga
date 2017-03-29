/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#ifndef SIMPLE_GA_H
#define SIMPLE_GA_H

#include <controllers/kilobot_clustering/kilobot_clustering.h>

#include <QString>

class CSimpleGA
{

public:
    CSimpleGA(std::vector<CKilobotClustering *> &ctrls, TConfigurationNode &t_node);

    void prepareNextGen();
    void loadNextGen();
    void flushIndividuals(const int curGeneration) const;
    float getGlobalPerformance() const;

private:
    typedef std::vector<LUTMotor> Population;

    std::vector<CKilobotClustering*> m_controllers;
    CRandom::CRNG* m_pcRNG;

    int m_iPopSize;
    int m_iMaxGenerations;
    int m_iTournamentSize;
    float m_fMutationRate;
    float m_fCrossoverRate;

    int m_iCurGeneration;
    bool m_bStoreData;
    QString m_sRelativePath;

    Population m_nextGen;

    uint32_t getBestRobotId();

    int tournamentSelection();


};

#endif // SIMPLE_GA_H
