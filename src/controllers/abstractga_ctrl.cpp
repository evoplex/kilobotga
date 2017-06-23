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

#include "abstractga_ctrl.h"

AbstractGACtrl::AbstractGACtrl()
    : m_pcRNG(CRandom::CreateRNG("kilobotga"))
    , m_pcMotors(NULL)
    , m_pcSensorOut(NULL)
    , m_pcSensorIn(NULL)
    , m_kMaxDistance(100)
    , m_kMinDistance(34)
    , m_kMaxForwardTicks(15)
    , m_kMaxTurningTicks(10)
    , m_fPerformance(0.f)
    , m_iCurrentTick(0)
    , m_iNextMotionTick(0)
    , m_currentMotion(STOP)
{
}

void AbstractGACtrl::Init(TConfigurationNode& t_node)
{
    m_pcMotors = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcSensorOut = GetActuator<CCI_KilobotCommunicationActuator>("kilobot_communication");
    m_pcSensorIn = GetSensor<CCI_KilobotCommunicationSensor>("kilobot_communication");
    m_pcLED = GetActuator<CCI_LEDsActuator>("leds");
}

void AbstractGACtrl::Reset()
{
    m_fPerformance = 0.f;
}

void AbstractGACtrl::setMotion(Motion motion)
{
    Real left = 0.f;
    Real right = 0.f;

    switch (motion) {
        case FORWARD: {
            left = 1.f;
            right = 1.f;
            break;
        }
        case TURN_LEFT: {
            right = 1.f;
            break;
        }
        case TURN_RIGHT: {
            left = 1.f;
            break;
        }
        case RAND_SPEEDS: {
            const CRange<Real> speedRange(0, 1);
            left = m_pcRNG->Uniform(speedRange);
            right = m_pcRNG->Uniform(speedRange);
            break;
        }
        case STOP:
        default:
            break;
    }

    m_currentMotion = motion;
    m_pcMotors->SetLinearVelocity(left * SPEED_SCALE, right * SPEED_SCALE);
}

void AbstractGACtrl::randWalk()
{
    m_iCurrentTick++;
    if (m_iCurrentTick < m_iNextMotionTick) {
        return;
    }

    if (m_currentMotion == FORWARD) {
        // flip coin: left or right?
        setMotion(m_pcRNG->Bernoulli() ? TURN_LEFT : TURN_RIGHT);
        m_iNextMotionTick = m_iCurrentTick + m_pcRNG->Uniform(CRange<UInt32>(1, m_kMaxTurningTicks));
    } else {
        setMotion(FORWARD);
        m_iNextMotionTick = m_iCurrentTick + m_pcRNG->Uniform(CRange<UInt32>(1, m_kMaxForwardTicks));
    }
}
