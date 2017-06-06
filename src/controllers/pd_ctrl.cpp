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

#include "pd_ctrl.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <QString>

PDCtrl::PDCtrl()
    : AbstractGACtrl()
    , m_pcRNG(CRandom::CreateRNG("kilobotga"))
{
    Reset();
}

void PDCtrl::Init(TConfigurationNode &t_node)
{
    AbstractGACtrl::Init(t_node);
    Reset();
}

void PDCtrl::Reset()
{
    AbstractGACtrl::Reset();

    // pure game strategy,
    // i.e., 0 (cooperate), 1 (defect) or 2 (abstain)
    Chromosome chromosome;
    chromosome.reserve(1);
    chromosome.push_back(randGene());
    setChromosome(chromosome);
}

void PDCtrl::ControlStep()
{
    // send message with my strategy
    m_pcSensorOut->SetMessage(&m_message);

    // read messages
    CCI_KilobotCommunicationSensor::TPackets in = m_pcSensorIn->GetPackets();

    // for each signal received, accumulate the payoff
    // obtained through the game interaction
    for (uint32_t i = 0; i < in.size(); ++i) {
        uint8_t strategyB = in[i].Message->data[0];
        m_fPerformance += calcPerformance(m_curStrategy, strategyB); // update performance
    }

    // update speed
    const CRange<Real> speedRange(0, 1);
    m_pcMotors->SetLinearVelocity(m_pcRNG->Uniform(speedRange) * SPEED_SCALE,
                                  m_pcRNG->Uniform(speedRange) * SPEED_SCALE);

    // led color
    m_pcLED->SetAllColors(m_curColor);
}

QVariant PDCtrl::randGene() const
{
    // pure strategy: 0 (C), 1 (D) or 2 (A)
    int strategy = m_pcRNG->Uniform(CRange<UInt32>(0, 3));
    return QVariant::fromValue(strategy);
}

bool PDCtrl::setChromosome(Chromosome chromosome)
{
    // chromosome holds one gene, which is the pure game strategy
    if (chromosome.size() != 1) {
        qFatal("\n[FATAL] Chromosome should have only one gene! (%ld)", chromosome.size());
        return false;
    }

    m_chromosome = chromosome;
    m_curStrategy = (uint8_t) chromosome[0].toInt();
    m_message.data[0] = m_curStrategy;

    switch (m_curStrategy) {
    case 0: // C
        m_curColor = CColor::BLUE;
        break;
    case 1: // D
        m_curColor = CColor::RED;
        break;
    case 2: // A
        m_curColor = CColor::GREEN;
        break;
    default:
        m_curColor = CColor::BROWN;
    }

    return true;
}

float PDCtrl::calcPerformance(uint8_t sA, uint8_t sB)
{
    if (sA == 2 || sB == 2) { // abstain
        return 2;
    }

    switch (sA*2+sB) {
    case 0: // CC
        return 3;
    case 1: // CD
        return 0;
    case 2: // DC
        return 5;
    case 3: // DD
        return 1;
    default:
        qFatal("\n[FATAL] wrong inputs in calcPerformance(%d, %d)", sA, sB);
        return 0;
    }
}

REGISTER_CONTROLLER(PDCtrl, "kilobot_pd_controller")
