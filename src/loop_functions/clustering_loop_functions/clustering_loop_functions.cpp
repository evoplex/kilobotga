/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#include "clustering_loop_functions.h"
#include <sstream>
#include <vector>

CClusteringLoopFunctions::CClusteringLoopFunctions()
    : m_iPopSize(10)
    , m_pcRNG(NULL)
{
    m_pcRNG = CRandom::CreateRNG("kilobotga");
}

void CClusteringLoopFunctions::Init(TConfigurationNode& t_node)
{
    // Create the kilobots and get a reference to their controllers
    for (int id=0; id < m_iPopSize; ++id) {
        std::stringstream entityId;
        entityId << "kb" << id;
        // fcc is the controller id as set in the XML
        CKilobotEntity* kilobot = new CKilobotEntity(entityId.str(), "fcc");
        AddEntity(*kilobot);
        m_entities.push_back(kilobot);
        m_controllers.push_back(&dynamic_cast<CKilobotClustering&>(kilobot->GetControllableEntity().GetController()));
    }

    // TODO: we should get it from the XML
    m_arenaSideX = CRange<Real>(-0.5, 0.5);
    m_arenaSideY = CRange<Real>(-0.5, 0.5);

    GetNodeAttribute(t_node, "population_size", m_iPopSize);
    GetNodeAttribute(t_node, "generations", m_iMaxGenerations);
    GetNodeAttribute(t_node, "tournament_size", m_iTournamentSize);
    GetNodeAttribute(t_node, "mutation_rate", m_fMutationRate);
    GetNodeAttribute(t_node, "crossover_rate", m_fCrossoverRate);

    Reset();
}

void CClusteringLoopFunctions::Reset()
{
    CQuaternion orientation;
    CVector3 position;
    const int maxPosTrial = 100;

    // uniform distribution
    for (uint32_t i = 0; i < m_entities.size(); ++i) {
        bool objAdded = false;
        for (int posTrial = 0; posTrial < maxPosTrial; ++posTrial) {
            CRadians zAngle = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
            orientation.FromEulerAngles(zAngle, CRadians::ZERO, CRadians::ZERO); // z, y, x
            position = CVector3(m_pcRNG->Uniform(m_arenaSideX), m_pcRNG->Uniform(m_arenaSideY), 0);

            if (MoveEntity(m_entities[i]->GetEmbodiedEntity(), position, orientation, false)) {
                objAdded = true;
                break;
            }
        }

        if (!objAdded) {
            LOGERR << "Unable to move robot to <" << position << ">, <" << orientation << ">" << std::endl;
        }
    }
}

uint32_t CClusteringLoopFunctions::getBestRobotId()
{
    int bestId = -1;
    float bestPerf = -1.f;
    for (uint32_t id = 0; id < m_controllers.size(); ++id) {
        float perf = m_controllers[id]->getPerformance();
        if (bestPerf < perf) {
            bestPerf = perf;
            bestId = id;
        }
    }
    return bestId;
}

float CClusteringLoopFunctions::getGlobalPerformance()
{
    float ret = 0.f;
    for (uint32_t id = 0; id < m_controllers.size(); ++id) {
        ret += m_controllers[id]->getPerformance();
    }
    return ret;
}

float CClusteringLoopFunctions::getPerformance(const uint32_t id)
{
    if (!robotExists(id)) {
        return 0.f;
    }
    return m_controllers[id]->getPerformance();
}

LUTMotor CClusteringLoopFunctions::getLUTMotor(uint32_t id)
{
    if (!robotExists(id)) {
        return LUTMotor();
    }
    return m_controllers[id]->getLUTMotor();
}

void CClusteringLoopFunctions::setLUTMotor(uint32_t id, LUTMotor lutMotor)
{
    if (robotExists(id)) {
        m_controllers[id]->setLUTMotor(lutMotor);
    }
}

bool CClusteringLoopFunctions::robotExists(const uint32_t id)
{
    if (id >= m_controllers.size()) {
        LOGERR << "The kilobot kb" << id << " does not exist!" << std::endl;
        return false;
    }
    return true;
}


REGISTER_LOOP_FUNCTIONS(CClusteringLoopFunctions, "clustering_loop_functions")
