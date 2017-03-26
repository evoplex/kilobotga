/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#include "kilobot_clustering.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

#define ALPHA -3 // d=[0,ALPHA]
#define MAX_LOCAL_PERFORMANCE 20 // max benefit

CKilobotClustering::CKilobotClustering() :
   m_pcMotors(NULL),
   m_pcSensorOut(NULL),
   m_pcSensorIn(NULL),
   m_kSpeedRange(0, 5),
   m_kMaxDistance(100),
   m_kMinDistance(34),
   m_iLUTSize(68),
   m_fPerformance(0.f),
   m_pcRNG(CRandom::CreateRNG("kilobotga"))
{
}

void CKilobotClustering::Init(TConfigurationNode& t_node) {
    m_pcMotors = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcSensorOut = GetActuator<CCI_KilobotCommunicationActuator>("kilobot_communication");
    m_pcSensorIn = GetSensor<CCI_KilobotCommunicationSensor>("kilobot_communication");

    // parse the configuration file
    GetNodeAttributeOrDefault(t_node, "lut_size", m_iLUTSize, m_iLUTSize);
    if(m_iLUTSize < 3) {
        LOGERR << "[FATAL] Invalid value for lut_size (" << m_iLUTSize << "). Should be a integer greater than 2." << std::endl;
    }

    Reset();
}

void CKilobotClustering::Reset() {
    initLUT();
    m_fPerformance = 0.f;
}

void CKilobotClustering::ControlStep()
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
            distance += in[i].Distance.high_gain;
        }
        distance /= in.size();
    } else {
        distance = m_kMaxDistance + 1; // no-signal
    }

    // update performance
    m_fPerformance += calcPerformance(getLUTIndex(distance));
    //LOG << m_fPerformance << std::endl;

    const Motor m = m_lutMotor[getLUTIndex(distance)];
    m_pcMotors->SetLinearVelocity(m.left, m.right);
}

void CKilobotClustering::initLUT()
{
    m_lutDistance.clear();
    m_lutMotor.clear();

    // first and last elements must hold the decision for MIN and MAX distance
    // i.e., [34, ... , no-signal]
    const int distInterval = round((m_kMaxDistance - m_kMinDistance) / (double)(m_iLUTSize - 2.0));
    int distance = m_kMinDistance;

    for (int i=0; i < m_iLUTSize; ++i) {
        Motor m;
        m.left = m_pcRNG->Uniform(m_kSpeedRange);
        m.right = m_pcRNG->Uniform(m_kSpeedRange);
        m_lutMotor.push_back(m);
        m_lutDistance.push_back(distance);
        //LOG << i << distance << m.left << m.right << std::endl;
        distance += distInterval;
    }
}

int CKilobotClustering::getLUTIndex(uint8_t distance)
{
    if (distance <= m_kMaxDistance) {
        for (int idx = 0; idx < m_lutDistance.size(); ++idx) {
            if (distance < m_lutDistance[idx])
                return idx;
        }
    }
    return m_lutDistance.size() - 1; // use last as default
}

float CKilobotClustering::calcPerformance(uint8_t distance)
{
    float x = distance + 1.f; // distance might be 0, so let's sum 1
    return MAX_LOCAL_PERFORMANCE * pow(x, ALPHA);
}


REGISTER_CONTROLLER(CKilobotClustering, "kilobot_clustering_controller")