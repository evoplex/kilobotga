/*
 * KilobotGA
 * Copyright (C) 2017 Marcos Cardinot <mcardinot@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "abstractga_lf.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/tinyxml/ticpp.h>

#include <QDebug>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>

AbstractGALoopFunction::AbstractGALoopFunction()
    : m_iPopSize(10)
    , m_iTournamentSize(2)
    , m_iMaxGenerations(1)
    , m_fMutationRate(0.f)
    , m_fCrossoverRate(0.f)
    , m_arenaSideX(0, 0)
    , m_arenaSideY(0, 0)
    , m_eSimMode(NEW_EXPERIMENT)
    , m_iCurGeneration(0)
{
    // create and seed our prg (using xml data)
    CRandom::CreateCategory("kilobotga", GetSimulator().GetRandomSeed());
    m_pcRNG = CRandom::CreateRNG("kilobotga");
}

void AbstractGALoopFunction::Init(TConfigurationNode& t_node)
{
    // retrieve settings from the '.argos' file
    GetNodeAttribute(t_node, "population_size", m_iPopSize);
    GetNodeAttribute(t_node, "generations", m_iMaxGenerations);
    GetNodeAttribute(t_node, "tournament_size", m_iTournamentSize);
    GetNodeAttribute(t_node, "mutation_rate", m_fMutationRate);
    GetNodeAttribute(t_node, "crossover_rate", m_fCrossoverRate);

    // TODO: we should get it from the XML too
    // we need the arena size to position the kilobots
    m_arenaSideX = CRange<Real>(-0.5, 0.5);
    m_arenaSideY = CRange<Real>(-0.5, 0.5);

    m_nextGeneration.reserve(m_iPopSize);

    // Create the kilobots and get a reference to their controllers
    for (uint32_t id = 0; id < m_iPopSize; ++id) {
        std::stringstream entityId;
        entityId << "kb" << id;
        // fcc is the controller id as set in the XML
        CKilobotEntity* kilobot = new CKilobotEntity(entityId.str(), "fcc");
        AddEntity(*kilobot);
        m_entities.push_back(kilobot);
        m_controllers.push_back(&dynamic_cast<AbstractGACtrl&>(kilobot->GetControllableEntity().GetController()));
    }

    // reset everything first
    Reset();

    // find out the simulation mode
    bool readFromFile;
    GetNodeAttribute(t_node, "read_from_file", readFromFile);
    if (readFromFile) {
        // if we're reading from files (i.e., reproducing an old experiment),
        // we need to load the LUT for each kilobot
        m_eSimMode = READ_EXPERIMENT;
        loadExperiment();
        qDebug() << "\nReading from file... \n";
    } else {
        QTextStream stream(stdin);
        int option = -1;
        while (option < 0 || option > 1) {
            qDebug() << "\nWhat do you want to do?\n"
                     << "\t 0 : Run a new experiment (no visualization) [DEFAULT] \n"
                     << "\t 1 : Test xml settings (visualize a single run)";
            option = stream.readLine().toInt();
        }

        m_eSimMode = (SIMULATION_MODE) option;
    }

    // if we are running a new experiment,
    // then we should prepare the directories
    if (m_eSimMode == NEW_EXPERIMENT) {
        // try to create a new directory to store our results
        m_sRelativePath = QDateTime::currentDateTime().toString("dd.MM.yy_hh.mm.ss");
        QDir dir(QDir::currentPath());
        if (!dir.mkdir(m_sRelativePath)) {
            qFatal("\n[FATAL] Unable to create a directory in %s\nResults will NOT be stored!\n",
                   qUtf8Printable(dir.absolutePath().append(m_sRelativePath)));
        } else if (dir.cd(m_sRelativePath)) {
            // create new folders for each generation
            for (uint32_t g = 0; g < m_iMaxGenerations; ++g) {
                dir.mkdir(QString::number(g));
            }

            // copy the .argos file
            SetNodeAttribute(t_node, "read_from_file", "true");
            t_node.GetDocument()->SaveFile(QString(m_sRelativePath + "/exp.argos").toStdString());
            SetNodeAttribute(t_node, "read_from_file", "false");

            // hide visualization during evolution
            t_node.GetDocument()->FirstChildElement()->FirstChildElement()->NextSiblingElement("visualization")->Clear();
        }
    }
}

void AbstractGALoopFunction::Reset()
{
    // make sure we reset our PRG before doing anything
    // it ensures that all kilobots will be back to the original position
    m_pcRNG->Reset();

    CQuaternion orientation;
    CVector3 position;
    const int maxPosTrial = 100;

    // uniform distribution (random)
    for (uint32_t i = 0; i < m_entities.size(); ++i) {
        bool objAdded = false;
        for (int posTrial = 0; posTrial < maxPosTrial; ++posTrial) {
            CRadians zAngle = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
            orientation.FromEulerAngles(zAngle, CRadians::ZERO, CRadians::ZERO); // z, y, x
            position = CVector3(m_pcRNG->Uniform(m_arenaSideX), m_pcRNG->Uniform(m_arenaSideY), 0);

            if (MoveEntity(m_entities[i]->GetEmbodiedEntity(), position, orientation, false)) {
                objAdded = true;
                break;
            }
        }

        if (!objAdded) {
            LOGERR << "Unable to move robot to <" << position << ">, <" << orientation << ">" << std::endl;
        }
    }
}

void AbstractGALoopFunction::PostExperiment()
{
    LOG << "Generation " << m_iCurGeneration << "\t"
        << getGlobalPerformance() << std::endl;

    if (m_eSimMode == NEW_EXPERIMENT) {
        flushGeneration();
        ++m_iCurGeneration;

        if (m_iCurGeneration < m_iMaxGenerations) {
            prepareNextGeneration();
            GetSimulator().Reset();

            loadNextGeneration();
            GetSimulator().Execute();
        }
    }
}

void AbstractGALoopFunction::prepareNextGeneration()
{
    m_nextGeneration.clear();

    // elitism: keep the best robot
    uint32_t bestId = getBestRobotId();
    m_nextGeneration.push_back(m_controllers[bestId]->getChromosome());

    const CRange<Real> zeroOne(0, 1);

    for (uint32_t i = 1; i < m_iPopSize; ++i) {
        // select two individuals
        uint32_t id1 = tournamentSelection();
        uint32_t id2 = tournamentSelection();
        // make sure they are different
        while (id1 == id2) id2 = tournamentSelection();

        Chromosome chromosome1 = m_controllers[id1]->getChromosome();
        Chromosome chromosome2 = m_controllers[id2]->getChromosome();
        Chromosome children = chromosome1;

        // crossover
        if (m_fCrossoverRate > 0.f) {
            for (uint32_t i = 0; i < chromosome1.size(); ++i) {
                if (m_pcRNG->Uniform(zeroOne) <= m_fCrossoverRate) {
                    children[i] = chromosome2[i];
                }
            }
        }

        // mutation
        if (m_fMutationRate > 0.f) {
            for (uint32_t i = 0; i < children.size(); ++i) {
                if (m_pcRNG->Uniform(zeroOne) <= m_fMutationRate) {
                    children[i] = m_controllers[id1]->randGene();
                }
            }
        }

        m_nextGeneration.push_back(children);
    }
}

void AbstractGALoopFunction::loadNextGeneration()
{
    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        m_controllers[kbId]->setChromosome(m_nextGeneration[kbId]);
    }
}

uint32_t AbstractGALoopFunction::tournamentSelection() const
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
    uint32_t bestPerfId = -1;
    for (uint32_t i = 0; i < ids.size(); ++i) {
        float perf = m_controllers[ids.at(i)]->getPerformance();
        if (perf > bestPerf) {
            bestPerf = perf;
            bestPerfId = ids.at(i);
        }
    }
    return bestPerfId;
}

float AbstractGALoopFunction::getGlobalPerformance() const
{
    float ret = 0.f;
    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        ret += m_controllers[kbId]->getPerformance();
    }
    return ret;
}

uint32_t AbstractGALoopFunction::getBestRobotId() const
{
    uint32_t bestId = -1;
    float bestPerf = -1.f;
    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        float perf = m_controllers[kbId]->getPerformance();
        if (bestPerf < perf) {
            bestPerf = perf;
            bestId = kbId;
        }
    }
    return bestId;
}
