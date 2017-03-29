/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#include "simplega.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/tinyxml/ticpp.h>

#include <QDateTime>
#include <QDir>
#include <fstream>

CSimpleGA::CSimpleGA(std::vector<CKilobotClustering*>& ctrls, TConfigurationNode& t_node)
    : m_controllers(ctrls)
    , m_pcRNG(CRandom::CreateRNG("kilobotga"))
    , m_bStoreData(false)
{
    GetNodeAttribute(t_node, "population_size", m_iPopSize);
    GetNodeAttribute(t_node, "generations", m_iMaxGenerations);
    GetNodeAttribute(t_node, "tournament_size", m_iTournamentSize);
    GetNodeAttribute(t_node, "mutation_rate", m_fMutationRate);
    GetNodeAttribute(t_node, "crossover_rate", m_fCrossoverRate);

    int maxGenerations = 0;
    bool readingFromFile = false;
    GetNodeAttribute(t_node, "generations", maxGenerations);
    GetNodeAttribute(t_node, "read_from_file", readingFromFile);

    m_nextGen.reserve(m_iPopSize);

    // if we are not loading an old experiment,
    // then we should prepare the directories
    if (!readingFromFile) {
        // try to create a new directory to store our results
        m_sRelativePath = QDateTime::currentDateTime().toString("dd.MM.yy_hh.mm.ss");
        QDir dir(QDir::currentPath());
        if (!dir.mkdir(m_sRelativePath)) {
            LOGERR << "Unable to create a directory in "
                   << dir.absolutePath().append(m_sRelativePath).toStdString()
                   << " Results will NOT be stored!" << std::endl;
        }

        // create new folders for each generation
        if (m_bStoreData && dir.cd(m_sRelativePath)) {
            for (int g = 0; g < maxGenerations; ++g) {
                dir.mkdir(QString::number(g));
            }
            // copy the .argos file
            t_node.GetDocument()->SaveFile(QString(m_sRelativePath + "/exp.argos").toStdString());
            // hide visualization during evolution
            t_node.GetDocument()->FirstChildElement()->FirstChildElement()->NextSiblingElement("visualization")->Clear();
        }

        m_bStoreData = true;
    }
}

void CSimpleGA::prepareNextGen()
{
    m_nextGen.clear();

    // elitism: keep the best robot
    uint32_t bestId = getBestRobotId();
    m_nextGen.push_back(m_controllers[bestId]->getLUTMotor());

    const CRange<Real> zeroOne(0, 1);

    for (int i = 1; i < m_iPopSize; ++i) {
        // select two individuals
        int id1 = tournamentSelection();
        int id2 = tournamentSelection();
        // make sure they are different
        while (id1 == id2) id2 = tournamentSelection();

        LUTMotor lutMotor1 = m_controllers[id1]->getLUTMotor();
        LUTMotor lutMotor2 = m_controllers[id2]->getLUTMotor();
        LUTMotor children = lutMotor1;

        // crossover
        if (m_fCrossoverRate > 0.f) {
            for (uint32_t i = 0; i < lutMotor1.size(); ++i) {
                if (m_pcRNG->Uniform(zeroOne) <= m_fCrossoverRate) {
                    children[i] = lutMotor2[i];
                }
            }
        }

        // mutation
        if (m_fMutationRate > 0.f) {
            for (uint32_t i = 0; i < children.size(); ++i) {
                if (m_pcRNG->Uniform(zeroOne) <= m_fMutationRate) {
                    Motor motor;
                    motor.left = m_pcRNG->Uniform(zeroOne);
                    motor.right = m_pcRNG->Uniform(zeroOne);
                    children[i] = motor;
                }
            }
        }

        m_nextGen.push_back(children);
    }
}

void CSimpleGA::loadNextGen()
{
    for (int kbId = 0; kbId < m_iPopSize; ++kbId) {
        m_controllers[kbId]->setLUTMotor(m_nextGen[kbId]);
    }
}

int CSimpleGA::tournamentSelection()
{
    // select random ids (make sure they are different)
    std::vector<uint32_t> ids;
    ids.reserve(m_iTournamentSize);
    while (ids.size() < m_iTournamentSize) {
        const uint32_t randId = m_pcRNG->Uniform(CRange<UInt32>(0, m_iPopSize));

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
        float perf = m_controllers[ids.at(i)]->getPerformance();
        if (perf > bestPerf) {
            bestPerf = perf;
            bestPerfId = ids.at(i);
        }
    }
    return bestPerfId;
}

void CSimpleGA::flushIndividuals(const int curGeneration) const
{
    if (!m_bStoreData) {
        return;
    }

    for (int kbId = 0; kbId < m_iPopSize; ++kbId) {
        QString path = QString("%1/%2/kb_%3.dat").arg(m_sRelativePath).arg(curGeneration).arg(kbId);
        std::ostringstream cOSS;
        cOSS << path.toStdString();
        std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);

        LUTMotor lutMotor = m_controllers[kbId]->getLUTMotor();
        for (uint32_t m = 0; m < lutMotor.size(); ++m) {
            cOFS << lutMotor[m].left << "\t" << lutMotor[m].right << std::endl;
        }
    }
}

float CSimpleGA::getGlobalPerformance() const
{
    float ret = 0.f;
    for (uint32_t kbId = 0; kbId < m_controllers.size(); ++kbId) {
        ret += m_controllers[kbId]->getPerformance();
    }
    return ret;
}

uint32_t CSimpleGA::getBestRobotId()
{
    int bestId = -1;
    float bestPerf = -1.f;
    for (uint32_t kbId = 0; kbId < m_controllers.size(); ++kbId) {
        float perf = m_controllers[kbId]->getPerformance();
        if (bestPerf < perf) {
            bestPerf = perf;
            bestId = kbId;
        }
    }
    return bestId;
}
