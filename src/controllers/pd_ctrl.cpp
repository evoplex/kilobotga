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
    // pure game strategy, i.e., 0 (cooperate) or 1 (defect)
    m_chromosome.reserve(1);

    // game strategy
    m_message.data[0] = m_pcRNG->Bernoulli();
}

void PDCtrl::ControlStep()
{
    // send my strategy
    m_pcSensorOut->SetMessage(&m_message);
    uint8_t sA = m_message.data[0];

    // read messages
    CCI_KilobotCommunicationSensor::TPackets in = m_pcSensorIn->GetPackets();

    // for each signal received, accumulate the payoff
    // obtained through the game interaction
    for (uint32_t i = 0; i < in.size(); ++i) {
        uint8_t sB = in[i].Message->data[0];
        m_fPerformance += calcPerformance(sA, sB); // update performance
    }

    // update speed
    const CRange<Real> speedRange(0, 1);
    m_pcMotors->SetLinearVelocity(m_pcRNG->Uniform(speedRange) * SPEED_SCALE,
                                  m_pcRNG->Uniform(speedRange) * SPEED_SCALE);

    // led color
    m_pcLED->SetAllColors(sA ? CColor::RED : CColor::BLUE);
}

QVariant PDCtrl::randGene() const
{
    int strategy = m_pcRNG->Bernoulli(); // 0 (C) or 1 (D)
    return QVariant::fromValue(strategy);
}

bool PDCtrl::setChromosome(Chromosome chromosome)
{
    // chromosome holds one gene, which is the pure game strategy
    if (chromosome.size() != 1) {
        qFatal("\n[FATAL] Chromosome should have only one gene!");
        return false;
    }
    m_chromosome = chromosome;
    return true;
}

float PDCtrl::calcPerformance(uint8_t sA, uint8_t sB)
{
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
        return 0;
    }
}

REGISTER_CONTROLLER(PDCtrl, "kilobot_pd_controller")
