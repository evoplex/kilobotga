/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#include "clustering_loop_functions.h"

#include <QDebug>
#include <QDir>
#include <QTextStream>
#include <sstream>
#include <vector>

CClusteringLoopFunctions::CClusteringLoopFunctions()
    : m_cGA(NULL)
    , m_eSimType(TEST_SETTINGS)
    , m_iPopSize(10)
    , m_iCurGeneration(0)
{
    // create and seed our prg (using xml data)
    CRandom::CreateCategory("kilobotga", GetSimulator().GetRandomSeed());
    m_pcRNG = CRandom::CreateRNG("kilobotga");
}

void CClusteringLoopFunctions::Init(TConfigurationNode& t_node)
{
    // retrieve a few settings from the '.argos' file
    // other stuff will be retrieved by the CSimpleGA class
    GetNodeAttribute(t_node, "population_size", m_iPopSize);
    GetNodeAttribute(t_node, "generations", m_iMaxGenerations);
    GetNodeAttribute(t_node, "simulation_type", m_eSimType);

    // TODO: we should get it from the XML too
    m_arenaSideX = CRange<Real>(-0.5, 0.5);
    m_arenaSideY = CRange<Real>(-0.5, 0.5);

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

    // reset everything first
    Reset();

    // create the GA object
    m_cGA = new CSimpleGA(m_controllers, t_node);

    // if we're just reading from files (i.e., reproducing an old experiment),
    // so we do not  to do any of the GA stuff, we just need to load the LUTs
    if (m_eSimType == REPLAY_EXPERIMENT) {
        QDir dir;
        int g = -1;
        while (g < 0) {
            QTextStream stream(stdin);
            bool ok = false;
            while (!ok) {
                qDebug() << "Which generation do you want to see? ";
                g = stream.readLine().toInt(&ok);
            }

            // we assume that we are within the experiment folder
            QFileInfo path(QString::fromStdString(GetSimulator().GetExperimentFileName()));
            dir = path.absoluteDir();
            dir.cd(QString::number(g));
            dir.setNameFilters(QStringList("kb*.dat"));
            dir.setFilter(QDir::Files | QDir::NoSymLinks);

            if (!dir.exists()) {
                qWarning() << "There is no data for this generation!";
                g = -1;
                continue;
            }

            // also check population size (number of files)
            if (dir.entryInfoList().size() != m_iPopSize) {
                qWarning() << "The folder for this generation should have "
                       << m_iPopSize << " files!";
                g = -1;
                continue;
            }
        }

        // all is fine, let's load the LUTs of each kilobot
        for (int kbId = 0; kbId < m_iPopSize; ++kbId) {
            QString fn = QString("kb_%1.dat").arg(kbId);
            loadLUTMotor(kbId, dir.absoluteFilePath(fn));
        }
    }
}

void CClusteringLoopFunctions::Reset()
{
    // make sure we reset our PRG before doing anything
    // it ensures that all kilobots will be back to the original position
    m_pcRNG->Reset();

    CQuaternion orientation;
    CVector3 position;
    const int maxPosTrial = 100;

    // uniform distribution (random)
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

void CClusteringLoopFunctions::PostExperiment()
{
    LOG << "Generation " << m_iCurGeneration << "\t"
        << m_cGA->getGlobalPerformance() << std::endl;

    if (!m_bReadFromFile) {
        m_cGA->flushIndividuals(m_iCurGeneration);
        ++m_iCurGeneration;

        if (m_iCurGeneration < m_iMaxGenerations) {
            m_cGA->prepareNextGen();
            GetSimulator().Reset();

            m_cGA->loadNextGen();
            GetSimulator().Execute();
        }
    }
}

bool CClusteringLoopFunctions::loadLUTMotor(int kbId, QString absoluteFilePath)
{
    // read file
    QFile file(absoluteFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        LOGERR << "[FATAL] Unable to open "
               << absoluteFilePath.toStdString() << std::endl;
        return false;
    }

    // load the lookup table
    LUTMotor lutMotor;
    QTextStream in(&file);
    while (!in.atEnd()) {
        QStringList values = in.readLine().split("\t");
        bool ok1, ok2;
        Motor m;
        m.left = QString(values.at(0)).toDouble(&ok1);
        m.right = QString(values.at(0)).toDouble(&ok2);
        if (!ok1 || !ok2 || values.size() != 2) {
            LOGERR << "[FATAL] Wrong values in " << absoluteFilePath.toStdString() << std::endl;
            return false;
        }
        lutMotor.push_back(m);
    }

    // check for lut size.
    // Must be equal to what we have in the .argos script
    if (lutMotor.size() != m_controllers[kbId]->getLUTSize()) {
        LOGERR << "[FATAL] Acconding to the XML file, the LUT size for " << absoluteFilePath.toStdString()
               << "should be " << m_controllers[0]->getLUTSize() << std::endl;
        return false;
    }

    // all is fine, setting the lookup table
    m_controllers[kbId]->setLUTMotor(lutMotor);
    return true;
}


REGISTER_LOOP_FUNCTIONS(CClusteringLoopFunctions, "clustering_loop_functions")
