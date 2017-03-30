/*
 * Marcos Cardinot <mcardinot@gmail.com>
 */

#include "clustering_loop_functions.h"

#include <QDebug>
#include <QDateTime>
#include <QDir>
#include <QTextStream>
#include <sstream>
#include <vector>

CClusteringLoopFunctions::CClusteringLoopFunctions()
    : m_cGA(NULL)
    , m_eSimMode(NEW_EXPERIMENT)
    , m_iCurGeneration(0)
    , m_iPopSize(10)
    , m_iMaxGenerations(1)
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

    // TODO: we should get it from the XML too
    // we need the arena size to position the kilobots
    m_arenaSideX = CRange<Real>(-0.5, 0.5);
    m_arenaSideY = CRange<Real>(-0.5, 0.5);

    // Create the kilobots and get a reference to their controllers
    for (uint32_t id = 0; id < m_iPopSize; ++id) {
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

    // find out the simulation mode
    bool readFromFile;
    GetNodeAttribute(t_node, "read_from_file", readFromFile);
    if (readFromFile) {
        // if we're reading from files (i.e., reproducing an old experiment),
        // we need to load the LUT for each kilobot
        m_eSimMode = READ_EXPERIMENT;
        loadExperiment();
        qDebug() << "\nReading from file... \n";
    } else {
        QTextStream stream(stdin);
        int option = -1;
        while (option < 0 || option > 1) {
            qDebug() << "\nWhat do you want to do?\n"
                     << "\t 0 : Run a new experiment (no visualization) [DEFAULT] \n"
                     << "\t 1 : Test xml settings (visualize a single run)";
            option = stream.readLine().toInt();
        }

        m_eSimMode = (SIMULATION_MODE) option;
    }

    // if we are loading a new experiment,
    // then we should prepare the directories
    if (m_eSimMode == CClusteringLoopFunctions::NEW_EXPERIMENT) {
        // try to create a new directory to store our results
        m_sRelativePath = QDateTime::currentDateTime().toString("dd.MM.yy_hh.mm.ss");
        QDir dir(QDir::currentPath());
        if (!dir.mkdir(m_sRelativePath)) {
            LOGERR << "[FATAL] Unable to create a directory in "
                   << dir.absolutePath().append(m_sRelativePath).toStdString()
                   << " Results will NOT be stored!" << std::endl;
            Q_ASSERT(false);
        } else if (dir.cd(m_sRelativePath)) {
            // create new folders for each generation
            for (uint32_t g = 0; g < m_iMaxGenerations; ++g) {
                dir.mkdir(QString::number(g));
            }

            // copy the .argos file
            SetNodeAttribute(t_node, "read_from_file", "true");
            t_node.GetDocument()->SaveFile(QString(m_sRelativePath + "/exp.argos").toStdString());
            SetNodeAttribute(t_node, "read_from_file", "false");

            // hide visualization during evolution
            t_node.GetDocument()->FirstChildElement()->FirstChildElement()->NextSiblingElement("visualization")->Clear();
        }
    }

    // create the GA object
    m_cGA = new CSimpleGA(m_controllers, t_node);
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

    if (m_eSimMode == NEW_EXPERIMENT) {
        m_cGA->flushIndividuals(m_sRelativePath, m_iCurGeneration);
        ++m_iCurGeneration;

        if (m_iCurGeneration < m_iMaxGenerations) {
            m_cGA->prepareNextGen();
            GetSimulator().Reset();

            m_cGA->loadNextGen();
            GetSimulator().Execute();
        }
    }
}

void CClusteringLoopFunctions::loadLUTMotor(const uint32_t kbId, const QString& absoluteFilePath)
{
    // read file
    QFile file(absoluteFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qFatal("[FATAL] Unable to open %s", qUtf8Printable(absoluteFilePath));
    }

    // load the lookup table
    LUTMotor lutMotor;
    QTextStream in(&file);
    while (!in.atEnd()) {
        QStringList values = in.readLine().split("\t");
        bool ok1, ok2;
        Motor m;
        m.left = QString(values.at(0)).toDouble(&ok1);
        m.right = QString(values.at(1)).toDouble(&ok2);
        if (!ok1 || !ok2 || values.size() != 2) {
            qFatal("\n[FATAL] Wrong values in %s", qUtf8Printable(absoluteFilePath));
        }
        lutMotor.push_back(m);
    }

    // check for lut size.
    // Must be equal to what we have in the .argos script
    if (lutMotor.size() != m_controllers[kbId]->getLUTSize()) {
        qFatal("\n[FATAL] Acconding to the XML file, the LUT size for %s should be %ld",
               qUtf8Printable(absoluteFilePath), m_controllers[0]->getLUTSize());
    }

    // all is fine, setting the lookup table
    m_controllers[kbId]->setLUTMotor(lutMotor);
}

void CClusteringLoopFunctions::loadExperiment()
{
    QTextStream stream(stdin);
    bool ok = false;
    while (!ok) {
        qDebug() << "\nWhich generation do you want to see? ";
        m_iCurGeneration = stream.readLine().toInt(&ok);
    }

    // we assume that we are within the experiment folder
    QFileInfo path(QString::fromStdString(GetSimulator().GetExperimentFileName()));
    QDir dir = path.absoluteDir();
    if (!dir.cd(QString::number(m_iCurGeneration))) {
        qFatal("\n[FATAL] There is no data for this generation!\n%s\n", qUtf8Printable(dir.absolutePath()));
    }

    // also check population size (number of files)
    dir.setNameFilters(QStringList("kb*.dat"));
    dir.setFilter(QDir::Files | QDir::NoSymLinks);
    if ((uint32_t) dir.entryInfoList().size() != m_iPopSize) {
        qFatal("\n[FATAL] The folder for this generation should have %ld files!\n%s\n",
               m_iPopSize, qUtf8Printable(dir.absolutePath()));
    }

    // all is fine, let's load the LUTs of each kilobot
    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        QString fn = QString("kb_%1.dat").arg(kbId);
        loadLUTMotor(kbId, dir.absoluteFilePath(fn));
    }
}


REGISTER_LOOP_FUNCTIONS(CClusteringLoopFunctions, "clustering_loop_functions")
