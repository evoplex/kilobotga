/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#ifndef KILOBOT_CLUSTERING_H
#define KILOBOT_CLUSTERING_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_sensor.h>

#define SPEED_PRECISION 10

using namespace argos;

// Motor speed [0, 1)
typedef struct {
   Real left;
   Real right;
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

    inline const float& getPerformance() const { return m_fPerformance; }
    inline const size_t& getLUTSize() const { return m_iLUTSize; }
    inline const LUTMotor& getLUTMotor() const { return m_lutMotor; }
    inline void setLUTMotor(LUTMotor lutMotor) { m_lutMotor = lutMotor; }

private:
     // actuators and sensors
     CCI_DifferentialSteeringActuator* m_pcMotors;
     CCI_KilobotCommunicationActuator* m_pcSensorOut;
     CCI_KilobotCommunicationSensor* m_pcSensorIn;

     // constants
     const uint8_t m_kMaxDistance;    // maximum distance from another robot in mm
     const uint8_t m_kMinDistance;    // minimum distance from another robot in mm

     // lookup tables
     size_t m_iLUTSize;
     LUTMotor m_lutMotor;
     std::vector<uint8_t> m_lutDistance;

     // behavioural state
     float m_fPerformance; // global performance of this kilobot

     // other stuff
     CRandom::CRNG*  m_pcRNG; // random number generator

     // initialize our lookup tables with random values
     void initLUT();

     // get a lut index from a distance (in mm)
     size_t getLUTIndex(uint8_t distance);

     // calculate the local performance
     // power-law: a*(x+1)^b.
     float calcPerformance(uint8_t distance);
};

#endif
