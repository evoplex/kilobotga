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

#ifndef SIMPLE_GA_H
#define SIMPLE_GA_H

#include "controllers/demo_ctrl.h"
#include "loop_functions/demo_lf.h"

#include <QString>

/**
 * @brief The SimpleGA class
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class SimpleGA
{

public:
    SimpleGA(std::vector<DemoCtrl*> &ctrls, TConfigurationNode &t_node);

    void prepareNextGen();
    void loadNextGen();
    void flushIndividuals(const QString &relativePath, const uint32_t curGeneration) const;
    float getGlobalPerformance() const;

private:
    typedef std::vector<LUTMotor> Population;

    std::vector<DemoCtrl*> m_controllers;
    CRandom::CRNG* m_pcRNG;

    const size_t m_iPopSize;
    size_t m_iTournamentSize;
    float m_fMutationRate;
    float m_fCrossoverRate;

    Population m_nextGen;

    uint32_t getBestRobotId();
    uint32_t tournamentSelection();
};

#endif // SIMPLE_GA_H
