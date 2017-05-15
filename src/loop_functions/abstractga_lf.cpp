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

#include <QFile>
#include <QString>
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
}

void AbstractGALoopFunction::PostExperiment()
{
    LOG << "Generation " << m_iCurGeneration << "\t"
        << getGlobalPerformance() << std::endl;

    if (m_eSimMode == NEW_EXPERIMENT) {
        flushIndividuals();
        ++m_iCurGeneration;

        if (m_iCurGeneration < m_iMaxGenerations) {
            prepareNextGen();
            GetSimulator().Reset();

            loadNextGen();
            GetSimulator().Execute();
        }
    }
}

void AbstractGALoopFunction::prepareNextGen()
{
    m_nextGeneration.clear();

    // elitism: keep the best robot
    uint32_t bestId = getBestRobotId();
    m_nextGeneration.push_back(m_controllers[bestId]->getLUTMotor());

    const CRange<Real> zeroOne(0, 1);

    for (uint32_t i = 1; i < m_iPopSize; ++i) {
        // select two individuals
        uint32_t id1 = tournamentSelection();
        uint32_t id2 = tournamentSelection();
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
                    Motor m;
                    m.left = QString::number(m_pcRNG->Uniform(zeroOne),'g', SPEED_PRECISION).toDouble();
                    m.right = QString::number(m_pcRNG->Uniform(zeroOne), 'g', SPEED_PRECISION).toDouble();
                    children[i] = m;
                }
            }
        }

        m_nextGeneration.push_back(children);
    }
}

void AbstractGALoopFunction::loadNextGen()
{
    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        m_controllers[kbId]->setLUTMotor(m_nextGeneration[kbId]);
    }
}

uint32_t AbstractGALoopFunction::tournamentSelection()
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

void AbstractGALoopFunction::flushIndividuals() const
{
    if (m_sRelativePath.isEmpty()) {
        qFatal("[FATAL] Unable to write! Directory was not defined!");
        return;
    }

    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        QString path = QString("%1/%2/kb_%3.dat").arg(m_sRelativePath).arg(m_iCurGeneration).arg(kbId);
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            qFatal("[FATAL] Unable to write in %s", qUtf8Printable(path));
        }

        QTextStream out(&file);
        out.setRealNumberPrecision(SPEED_PRECISION);
        LUTMotor lutMotor = m_controllers[kbId]->getLUTMotor();
        for (uint32_t m = 0; m < lutMotor.size(); ++m) {
            out << lutMotor[m].left << "\t" << lutMotor[m].right << "\n";
        }
    }
}

float AbstractGALoopFunction::getGlobalPerformance() const
{
    float ret = 0.f;
    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        ret += m_controllers[kbId]->getPerformance();
    }
    return ret;
}

uint32_t AbstractGALoopFunction::getBestRobotId()
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
