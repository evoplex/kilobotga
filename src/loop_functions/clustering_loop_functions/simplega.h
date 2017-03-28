#ifndef SIMPLE_GA_H
#define SIMPLE_GA_H

#include <controllers/kilobot_clustering/kilobot_clustering.h>

class CSimpleGA
{

public:
    CSimpleGA(std::vector<CKilobotClustering *> &ctrls, TConfigurationNode &t_node);

private:
    std::vector<CKilobotClustering*> m_controllers;
    CRandom::CRNG* m_pcRNG;

    int m_iPopSize;
    int m_iMaxGenerations;
    int m_iTournamentSize;
    float m_fMutationRate;
    float m_fCrossoverRate;

    int m_iCurGeneration;

};

#endif // SIMPLE_GA_H
