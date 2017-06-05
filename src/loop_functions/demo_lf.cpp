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

#include "demo_lf.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QTextStream>

DemoLF::DemoLF()
    : AbstractGALoopFunction()
{
}

void DemoLF::flushGeneration() const
{
    if (m_sRelativePath.isEmpty()) {
        qFatal("[FATAL] Unable to write! Directory was not defined!");
        return;
    }

    for (uint32_t kbId = 0; kbId < m_iPopSize; ++kbId) {
        QString path = QString("%1/%2/kb_%3.dat").arg(m_sRelativePath).arg(m_iCurGeneration).arg(kbId);
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            qFatal("[FATAL] Unable to write in %s", qUtf8Printable(path));
        }

        QTextStream out(&file);
        out.setRealNumberPrecision(SPEED_PRECISION);
        Chromosome chromosome = m_controllers[kbId]->getChromosome();

        for (uint32_t m = 0; m < chromosome.size(); ++m) {
            MotorSpeed motorSpeed = chromosome[m].value<MotorSpeed>();
            out << motorSpeed.left << "\t" << motorSpeed.right << "\n";
        }
    }
}

void DemoLF::loadExperiment()
{
    QTextStream stream(stdin);
    bool ok = false;
    while (!ok) {
        qDebug() << "\nWhich generation do you want to see? ";
        m_iCurGeneration = stream.readLine().toInt(&ok);
    }

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

void DemoLF::loadLUTMotor(const uint32_t kbId, const QString& absoluteFilePath) const
{
    // read file
    QFile file(absoluteFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qFatal("[FATAL] Unable to open %s", qUtf8Printable(absoluteFilePath));
    }

    // load the lookup table
    Chromosome chromosome;
    QTextStream in(&file);
    while (!in.atEnd()) {
        QStringList values = in.readLine().split("\t");
        bool ok1, ok2;
        MotorSpeed m;
        m.left = QString(values.at(0)).toDouble(&ok1);
        m.right = QString(values.at(1)).toDouble(&ok2);
        if (!ok1 || !ok2 || values.size() != 2) {
            qFatal("\n[FATAL] Wrong values in %s", qUtf8Printable(absoluteFilePath));
        }
        chromosome.push_back(QVariant::fromValue(m));
    }

    // all is fine, setting the lookup table
    if (!m_controllers[kbId]->setChromosome(chromosome)) {
        // something went wrong; print filepath
        qFatal("\n[FATAL] Something went wrong when loading the chromosome values: %s", qUtf8Printable(absoluteFilePath));
    }
}

REGISTER_LOOP_FUNCTIONS(DemoLF, "demo_loop_functions")
