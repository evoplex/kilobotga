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

#include "demo_ctrl.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <QString>

// parameters of our fitness function
#define ALPHA 3 // begning of the long tail
#define MAX_LOCAL_PERFORMANCE 20 // max score received in one interaction

DemoCtrl::DemoCtrl()
    : AbstractGACtrl()
    , m_iLUTSize(68)
{
}

void DemoCtrl::Init(TConfigurationNode& t_node)
{
    AbstractGACtrl::Init(t_node);

    // parse the configuration file
    GetNodeAttributeOrDefault(t_node, "lut_size", m_iLUTSize, m_iLUTSize);
    if(m_iLUTSize < 3) {
        LOGERR << "[FATAL] Invalid value for lut_size (" << m_iLUTSize
               << "). Should be a integer greater than 2." << std::endl;
    }

    m_lutDistance.reserve(m_iLUTSize);
    m_chromosome.reserve(m_iLUTSize);

    Reset();
}

void DemoCtrl::Reset()
{
    initLUT();
    m_fPerformance = 0.f;
}

void DemoCtrl::ControlStep()
{
    // send an empty message
    m_pcSensorOut->SetMessage(NULL);

    // read messages
    CCI_KilobotCommunicationSensor::TPackets in = m_pcSensorIn->GetPackets();

    // Handling signals received
    // if received more than 1 message, take the average distance
    // otherwise, use the max+1 distance (no-signal)
    uint8_t distance = 0;
    if (in.size()) {
        for (uint32_t i = 0; i < in.size(); ++i) {
            uint8_t d = in[i].Distance.high_gain;
            m_fPerformance += calcPerformance(getLUTIndex(d)); // update performance
            distance += d;
        }
        distance /= in.size();
    } else { // no message was received
        distance = m_kMaxDistance + 1;
    }

    // update speed
    const MotorSpeed m = m_chromosome[getLUTIndex(distance)].value<MotorSpeed>();
    m_pcMotors->SetLinearVelocity(m.left * SPEED_SCALE, m.right * SPEED_SCALE);
}

QVariant DemoCtrl::randGene() const
{
    const CRange<Real> speedRange(0, 1);
    MotorSpeed m;
    m.left = QString::number(m_pcRNG->Uniform(speedRange),'g', SPEED_PRECISION).toDouble();
    m.right = QString::number(m_pcRNG->Uniform(speedRange), 'g', SPEED_PRECISION).toDouble();
    return QVariant::fromValue(m);
}

bool DemoCtrl::setChromosome(Chromosome chromosome)
{
    // check for lut size. Must be equal to what we have in the .argos script
    if (chromosome.size() != m_iLUTSize) {
        qFatal("\n[FATAL] Acconding to the XML file, the LUT size should be %ld", m_iLUTSize);
        return false;
    }
    m_chromosome = chromosome;
    return true;
}

void DemoCtrl::initLUT()
{
    m_lutDistance.clear();
    m_chromosome.clear();

    // first and last elements must hold the decision for MIN and MAX distance
    // i.e., [34, ... , no-signal]
    const int distInterval = round((m_kMaxDistance - m_kMinDistance) / (double)(m_iLUTSize - 2.0));
    int distance = m_kMinDistance;

    for (uint32_t i = 0; i < m_iLUTSize; ++i) {
        m_chromosome.push_back(randGene());
        m_lutDistance.push_back(distance);
        distance += distInterval;
    }
}

size_t DemoCtrl::getLUTIndex(uint8_t distance) const
{
    if (distance <= m_kMaxDistance) {
        for (size_t idx = 0; idx < m_lutDistance.size(); ++idx) {
            if (distance < m_lutDistance[idx])
                return idx;
        }
    }
    return m_lutDistance.size() - 1; // use last as default
}

float DemoCtrl::calcPerformance(uint8_t distance) const
{
    float x = distance + 1.f; // distance might be 0, so let's sum 1
    return MAX_LOCAL_PERFORMANCE * pow(x, -ALPHA);
}

REGISTER_CONTROLLER(DemoCtrl, "kilobot_demo_controller")
