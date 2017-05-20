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
    : m_pcMotors(NULL)
    , m_pcSensorOut(NULL)
    , m_pcSensorIn(NULL)
    , m_kMaxDistance(100)
    , m_kMinDistance(34)
    , m_fPerformance(0.f)
{
}

void AbstractGACtrl::Init(TConfigurationNode& t_node)
{
    m_pcMotors = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcSensorOut = GetActuator<CCI_KilobotCommunicationActuator>("kilobot_communication");
    m_pcSensorIn = GetSensor<CCI_KilobotCommunicationSensor>("kilobot_communication");
}

void AbstractGACtrl::Reset()
{
    m_fPerformance = 0.f;
}
