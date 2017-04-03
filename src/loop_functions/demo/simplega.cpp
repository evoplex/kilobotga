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

#include "simplega.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/tinyxml/ticpp.h>

#include <QFile>
#include <QString>
#include <QTextStream>

SimpleGA::SimpleGA(std::vector<DemoCtrl*>& ctrls, TConfigurationNode& t_node)
    : m_controllers(ctrls)
    , m_pcRNG(CRandom::CreateRNG("kilobotga"))
{
    GetNodeAttribute(t_node, "population_size", m_iPopSize);
    GetNodeAttribute(t_node, "tournament_size", m_iTournamentSize);
    GetNodeAttribute(t_node, "mutation_rate", m_fMutationRate);
    GetNodeAttribute(t_node, "crossover_rate", m_fCrossoverRate);

    m_nextGen.reserve(m_iPopSize);
}

void SimpleGA::prepareNextGen()
{
    m_nextGen.clear();

    // elitism: keep the best robot
    uint32_t bestId = getBestRobotId();
    m_nextGen.push_back(m_controllers[bestId]->getLUTMotor());

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

        m_nextGen.push_back(children);
    }
}

void SimpleGA::loadNextGen()
{
    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        m_controllers[kbId]->setLUTMotor(m_nextGen[kbId]);
    }
}

uint32_t SimpleGA::tournamentSelection()
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

void SimpleGA::flushIndividuals(const QString& relativePath, const uint32_t curGeneration) const
{
    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        QString path = QString("%1/%2/kb_%3.dat").arg(relativePath).arg(curGeneration).arg(kbId);
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

float SimpleGA::getGlobalPerformance() const
{
    float ret = 0.f;
    for (uint32_t kbId = 0; kbId < m_controllers.size(); ++kbId) {
        ret += m_controllers[kbId]->getPerformance();
    }
    return ret;
}

uint32_t SimpleGA::getBestRobotId()
{
    uint32_t bestId = -1;
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