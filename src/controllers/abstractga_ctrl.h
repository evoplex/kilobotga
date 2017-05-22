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
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

#include <QVariant>

using namespace argos;

// speed is always a real number between 0 and 1 scaled by this constant
#define SPEED_SCALE 10

// we use this constant to avoid any propagation of uncertainty
// when reading/writting numbers from a file for example.
#define SPEED_PRECISION 10

// Motor speed [0, 1)
struct MotorSpeed {
   Real left;
   Real right;
};
Q_DECLARE_METATYPE(MotorSpeed)

typedef std::vector<QVariant> Chromosome;

/**
 * @brief The AbstractGACtrl class
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class AbstractGACtrl : public CCI_Controller
{

public:
    AbstractGACtrl();
    virtual ~AbstractGACtrl() {}

    // return false if chromosome is not suitable
    virtual bool setChromosome(Chromosome chromosome) = 0;

    // generate a random gene
    virtual QVariant randGene() const = 0;

    inline const Chromosome& getChromosome() const { return m_chromosome; }
    inline const float& getPerformance() const { return m_fPerformance; }

    // CCI_Controler stuff
    virtual void Init(TConfigurationNode& t_node);
    virtual void Reset();

protected:
    // actuators and sensors
    CCI_DifferentialSteeringActuator* m_pcMotors;
    CCI_KilobotCommunicationActuator* m_pcSensorOut;
    CCI_KilobotCommunicationSensor* m_pcSensorIn;
    CCI_LEDsActuator* m_pcLED;

    // constants
    const uint8_t m_kMaxDistance;    // maximum distance from another robot in mm
    const uint8_t m_kMinDistance;    // minimum distance from another robot in mm

    // genetic algorithm stuff
    float m_fPerformance; // global performance of this kilobot
    Chromosome m_chromosome;
};

#endif // ABSTRACTGA_CTRL_H
