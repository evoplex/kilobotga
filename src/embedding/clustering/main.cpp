/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <loop_functions/clustering_loop_functions/clustering_loop_functions.cpp>

// kilolib.h redefines the main() to __kilobot_main()
// this line is important to make it main() again
#define __kilobot_main main

typedef std::vector<LUTMotor> Population;

void flushLUT(const int id, const int generation, const LUTMotor& lutMotor)
{
    std::ostringstream cOSS;
    cOSS << "kb" << id << "_g" << generation << ".dat";
    std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
    for (int i = 0; i < lutMotor.size(); ++i) {
        cOFS << lutMotor.at(i).left << lutMotor.at(i).right << std::endl;
    }
}

int tournamentSelection(CClusteringLoopFunctions& loopFunction, CRandom::CRNG* prg)
{
    const int popSize = loopFunction.getPopSize();
    const size_t tournamentSize = loopFunction.getTournamentSize();

    // select random ids (we make sure they are different)
    std::vector<uint32_t> ids;
    ids.reserve(tournamentSize);
    while (ids.size() < tournamentSize) {
        const int randId = prg->Uniform(CRange<UInt32>(0, popSize));

        // check if randId has not already been chosen
        bool exists = false;
        for (int i = 0; i < ids.size(); ++i) {
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
    for (int i = 0; i < ids.size(); ++i) {
        float perf = loopFunction.getPerformance(ids.at(i));
        if (perf > bestPerf) {
            bestPerf = perf;
            bestPerfId = ids.at(i);
        }
    }
    return bestPerfId;
}

void newGeneration(CClusteringLoopFunctions& loopFunction, CRandom::CRNG* prg)
{
    // retrieve a few settings from the 'experiment.argos' file
    const int lutSize = loopFunction.getLUTSize();
    const int popSize = loopFunction.getPopSize();
    const int tournamentSize = loopFunction.getTournamentSize();
    const float mutationRate = loopFunction.getMutationRate();
    const float crossoverRate = loopFunction.getCrossoverRate();

    Population newPop;

    // elitism: keep the best robot
    uint32_t bestId = loopFunction.getBestRobotId();
    newPop.push_back(loopFunction.getLUTMotor(bestId));

    for (int i = 1; i < popSize; ++i) {
        // select two individuals
        int id1 = tournamentSelection(loopFunction, prg);
        int id2 = tournamentSelection(loopFunction, prg);
        // make sure they are different
        while (id1 == id2) id2 = tournamentSelection(loopFunction, prg);

        LUTMotor lutMotor1 = loopFunction.getLUTMotor(id1);
        LUTMotor lutMotor2 = loopFunction.getLUTMotor(id1);

        // crossover
        LUTMotor children = lutMotor1;
        for (int i = 0; i < lutMotor1.size(); ++i) {
            if (prg->Uniform(CRange<Real>(0, 1)) <= crossoverRate) {
                children[i] = lutMotor2[i];
            }
        }

        // mutation
        for (int i = 0; i < children.size(); ++i) {
            if (prg->Uniform(CRange<Real>(0, 1)) <= mutationRate) {
                Motor motor;
                motor.left = prg->Uniform(loopFunction.getSpeedRange());
                motor.right = prg->Uniform(loopFunction.getSpeedRange());
                children[i] = motor;
            }
        }

        newPop.push_back(children);
    }

    // run experiment for the new population
    static argos::CSimulator& simulator = argos::CSimulator::GetInstance();
    simulator.Reset();
    for (int id = 0; id < popSize; ++id) {
        loopFunction.setLUTMotor(id, newPop.at(id));
    }
    simulator.Execute();
}

int main(int argc, char** argv)
{
    CRandom::CreateCategory("kilobotga", 7751);
    CRandom::CRNG* prg = CRandom::CreateRNG("kilobotga");

    // initialize ARGoS
    static argos::CSimulator& simulator = argos::CSimulator::GetInstance();

    // load the experiment
    simulator.SetExperimentFileName("src/embedding/clustering/experiment.argos");
    simulator.LoadExperiment();

    // reference to the loop function
    static CClusteringLoopFunctions& loopFunction = dynamic_cast<CClusteringLoopFunctions&>(simulator.GetLoopFunctions());

    const int maxGenerations = loopFunction.getMaxGenerations();

    // run genetic algorithm
    simulator.Reset();
    simulator.Execute();
    int generation = 0;
    while(generation < maxGenerations) {
        argos::LOG << "G#" << generation << "..." << loopFunction.getGlobalPerformance() << std::endl;
        newGeneration(loopFunction, prg);
        ++generation;
    }

    // dispose of ARGoS stuff
    simulator.Destroy();

    return 0;
}

