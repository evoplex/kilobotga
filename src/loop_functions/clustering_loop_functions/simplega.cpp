#include "simplega.h"

CSimpleGA::CSimpleGA(std::vector<CKilobotClustering*>& ctrls, TConfigurationNode& t_node)
    : m_controllers(ctrls)
    , m_pcRNG(CRandom::CreateRNG("kilobotga"))
{
    GetNodeAttribute(t_node, "population_size", m_iPopSize);
    GetNodeAttribute(t_node, "generations", m_iMaxGenerations);
    GetNodeAttribute(t_node, "tournament_size", m_iTournamentSize);
    GetNodeAttribute(t_node, "mutation_rate", m_fMutationRate);
    GetNodeAttribute(t_node, "crossover_rate", m_fCrossoverRate);
}

Population CSimpleGA::newGeneration()
{
    // retrieve a few settings from the 'experiment.argos' file
    const int popSize = loopFunction.getPopSize();
    const float mutationRate = loopFunction.getMutationRate();
    const float crossoverRate = loopFunction.getCrossoverRate();

    Population newPop;
    newPop.reserve(popSize);

    // elitism: keep the best robot
    uint32_t bestId = loopFunction.getBestRobotId();
    newPop.push_back(loopFunction.getLUTMotor(bestId));

    const CRange<Real> zeroOne(0, 1);

    for (int i = 1; i < popSize; ++i) {
        // select two individuals
        int id1 = tournamentSelection(loopFunction, prg);
        int id2 = tournamentSelection(loopFunction, prg);
        // make sure they are different
        while (id1 == id2) id2 = tournamentSelection(loopFunction, prg);

        LUTMotor lutMotor1 = loopFunction.getLUTMotor(id1);
        LUTMotor lutMotor2 = loopFunction.getLUTMotor(id2);
        LUTMotor children = lutMotor1;

        // crossover
        if (crossoverRate > 0.f) {
            for (uint32_t i = 0; i < lutMotor1.size(); ++i) {
                if (prg->Uniform(zeroOne) <= crossoverRate) {
                    children[i] = lutMotor2[i];
                }
            }
        }

        // mutation
        if (mutationRate > 0.f) {
            for (uint32_t i = 0; i < children.size(); ++i) {
                if (prg->Uniform(zeroOne) <= mutationRate) {
                    Motor motor;
                    motor.left = prg->Uniform(loopFunction.getSpeedRange());
                    motor.right = prg->Uniform(loopFunction.getSpeedRange());
                    children[i] = motor;
                }
            }
        }

        newPop.push_back(children);
    }

    // run experiment for the new population
    static argos::CSimulator& simulator = argos::CSimulator::GetInstance();
    //CRandom::GetCategory("kilobotga").ResetRNGs();
    simulator.Reset();
    for (int id = 0; id < popSize; ++id) {
        loopFunction.setLUTMotor(id, newPop[id]);
    }
    simulator.Execute();

    return newPop;
}

int CSimpleGA::tournamentSelection()
{
    const int popSize = loopFunction.getPopSize();
    const size_t tournamentSize = loopFunction.getTournamentSize();

    // select random ids (we make sure they are different)
    std::vector<uint32_t> ids;
    ids.reserve(tournamentSize);
    while (ids.size() < tournamentSize) {
        const uint32_t randId = prg->Uniform(CRange<UInt32>(0, popSize));

        // check if randId has not already been chosen
        bool exists = false;
        for (uint32_t i = 0; i < ids.size(); ++i) {
            if (randId == ids[i]) {
                exists = true;
                break;
            }
        }

        if (!exists) {
            ids.push_back(randId);
        }
    }

    // get the fittest
    float bestPerf = -1;
    int bestPerfId = -1;
    for (uint32_t i = 0; i < ids.size(); ++i) {
        float perf = loopFunction.getPerformance(ids.at(i));
        if (perf > bestPerf) {
            bestPerf = perf;
            bestPerfId = ids.at(i);
        }
    }
    return bestPerfId;
}
