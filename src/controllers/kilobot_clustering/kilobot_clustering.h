/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#ifndef KILOBOT_CLUSTERING_H
#define KILOBOT_CLUSTERING_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_sensor.h>

#include <argos3/core/utility/math/rng.h> // Random number generator
#include <argos3/core/utility/logging/argos_log.h> // Logging functions

using namespace argos;

typedef struct {
   uint8_t left;
   uint8_t right;
} Motor;

typedef std::vector<Motor> LUTMotor;

/**
 * @brief The CKilobotClustering class
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class CKilobotClustering : public CCI_Controller
{

public:
    CKilobotClustering();
    virtual ~CKilobotClustering() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();
    virtual void Reset();
    virtual void Destroy() {}

    inline const CRange<UInt32>& getSpeedRange() const { return m_kSpeedRange; }
    inline const float& getPerformance() const { return m_fPerformance; }
    inline const int& getLUTSize() const { return m_iLUTSize; }
    inline const LUTMotor& getLUTMotor() const { return m_lutMotor; }
    inline void setLUTMotor(LUTMotor lutMotor) { m_lutMotor = lutMotor; }

private:
     // actuators and sensors
     CCI_DifferentialSteeringActuator* m_pcMotors;
     CCI_KilobotCommunicationActuator* m_pcSensorOut;
     CCI_KilobotCommunicationSensor* m_pcSensorIn;

     // constants
     const CRange<UInt32> m_kSpeedRange;
     const uint8_t m_kMaxDistance;    // maximum distance from another robot in mm
     const uint8_t m_kMinDistance;    // minimum distance from another robot in mm

     // lookup tables
     int m_iLUTSize;
     LUTMotor m_lutMotor;
     std::vector<uint8_t> m_lutDistance;

     // behavioural state
     float m_fPerformance; // global performance of this kilobot

     // other stuff
     CRandom::CRNG*  m_pcRNG; // random number generator

     // initialize our lookup tables with random values
     void initLUT();

     // get a lut index from a distance (in mm)
     int getLUTIndex(uint8_t distance);

     // calculate the local performance
     // power-law: a*(x+1)^b.
     float calcPerformance(uint8_t distance);
};

#endif
