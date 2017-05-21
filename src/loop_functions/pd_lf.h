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

#ifndef PD_LOOP_FUNCTIONS_H
#define PD_LOOP_FUNCTIONS_H

#include "abstractga_lf.h"
#include "controllers/pd_ctrl.h"

/**
 * @brief The PDLF class
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class PDLF : public AbstractGALoopFunction
{

public:
    PDLF();
    virtual ~PDLF() {}

    virtual void Reset();

private:
    CRandom::CRNG* m_pcRNG;

    virtual void flushGeneration() const;
    virtual void loadExperiment();
};

#endif // PD_LOOP_FUNCTIONS_H
