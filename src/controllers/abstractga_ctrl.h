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

#ifndef ABSTRACTGA_CTRL_H
#define ABSTRACTGA_CTRL_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_sensor.h>

using namespace argos;

// Motor speed [0, 1)
typedef struct {
   Real left;
   Real right;
} Motor;

typedef std::vector<Motor> LUTMotor;

#define SPEED_PRECISION 10

/**
 * @brief The AbstractGACtrl class
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class AbstractGACtrl : public CCI_Controller
{

public:
    AbstractGACtrl();
    virtual ~AbstractGACtrl() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep() {}
    virtual void Reset() {}
    virtual void Destroy() {}

    inline const float& getPerformance() const { return m_fPerformance; }
    inline const size_t& getLUTSize() const { return m_iLUTSize; }
    inline const LUTMotor& getLUTMotor() const { return m_lutMotor; }
    inline void setLUTMotor(LUTMotor lutMotor) { m_lutMotor = lutMotor; }

protected:
     // actuators and sensors
     CCI_DifferentialSteeringActuator* m_pcMotors;
     CCI_KilobotCommunicationActuator* m_pcSensorOut;
     CCI_KilobotCommunicationSensor* m_pcSensorIn;

     // constants
     const uint8_t m_kMaxDistance;    // maximum distance from another robot in mm
     const uint8_t m_kMinDistance;    // minimum distance from another robot in mm

     // behavioural state
     float m_fPerformance; // global performance of this kilobot

     // lookup tables
     size_t m_iLUTSize;
     LUTMotor m_lutMotor;
     std::vector<uint8_t> m_lutDistance;
};

#endif // ABSTRACTGA_CTRL_H
