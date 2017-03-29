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

#include <fstream>

LUTMotor loadLUTMotor(const std::string& str_filename, const uint8_t lutSize)
{
   // open the input file
   std::ifstream cIn(str_filename.c_str(), std::ios::in);
   if(!cIn) {
      THROW_ARGOSEXCEPTION("Cannot open parameter file '" << str_filename << "' for reading");
   }

   LUTMotor lutMotor;
   lutMotor.reserve(lutSize);
   for(size_t i = 0; i < lutSize; ++i) {
       Motor m;
      if(!(cIn >> m.left >> m.right)) {
         THROW_ARGOSEXCEPTION("Cannot read data from file '" << str_filename << "'");
      }
      lutMotor.push_back(m);
   }

   return lutMotor;
}

void loadPopulation(CClusteringLoopFunctions& loopFunction, const int generation)
{
    static argos::CSimulator& simulator = argos::CSimulator::GetInstance();

    const int lutSize = loopFunction.getLUTSize();
    const int popSize = loopFunction.getPopSize();

    simulator.Reset();
    for (int kb = 0; kb < popSize; ++kb) {
        std::ostringstream cOSS;
        cOSS << "kb" << kb << "_g" << generation << ".dat";
        loopFunction.setLUTMotor(kb, loadLUTMotor(cOSS.str(), lutSize));
    }
    simulator.Execute();
}


void flushLUT(const Population& pop, const int generation)
{
    for (uint32_t kb = 0; kb < pop.size(); ++kb) {
        std::ostringstream cOSS;
        cOSS << "kb" << kb << "_g" << generation << ".dat";
        std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
        LUTMotor lutMotor = pop[kb];
        for (uint32_t m = 0; m < lutMotor.size(); ++m) {
            cOFS << lutMotor[m].left << "\t" << lutMotor[m].right << std::endl;
        }
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

Population newGeneration(CClusteringLoopFunctions& loopFunction, CRandom::CRNG* prg)
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

int main(int argc, char** argv)
{
    // initialize ARGoS
    static argos::CSimulator& simulator = argos::CSimulator::GetInstance();

    // load the experiment
    CRandom::CreateCategory("kilobotga", 0); // reseed it later
    simulator.SetExperimentFileName("src/embedding/clustering/experiment.argos");
    simulator.LoadExperiment();

    // create and seed our prg (using xml data)
    CRandom::SetSeedOf("kilobotga", simulator.GetRandomSeed());
    CRandom::GetCategory("kilobotga").ResetRNGs();
    CRandom::CRNG* prg = CRandom::CreateRNG("kilobotga");

    // reference to the loop function
    static CClusteringLoopFunctions& loopFunction = dynamic_cast<CClusteringLoopFunctions&>(simulator.GetLoopFunctions());

    const int maxGenerations = loopFunction.getMaxGenerations();

    //loadPopulation(loopFunction, maxGenerations);

    // run genetic algorithm
    simulator.Reset();
    simulator.Execute();


/*
    Population pop;
    int generation = 0;
    while(generation < maxGenerations) {
        argos::LOG << generation << "\t" << loopFunction.getGlobalPerformance() << std::endl;
        pop = newGeneration(loopFunction, prg);
        ++generation;
    }
*/
    // flush last generation
    //flushLUT(pop, generation);

    // dispose of ARGoS stuff
    simulator.Destroy();

    return 0;
}

