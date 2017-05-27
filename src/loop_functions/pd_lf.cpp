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

#include "pd_lf.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QTextStream>

PDLF::PDLF()
    : AbstractGALoopFunction()
{
}

void PDLF::flushGeneration() const
{
// TODO
}

void PDLF::loadExperiment()
{
// TODO
}

REGISTER_LOOP_FUNCTIONS(PDLF, "pd_loop_functions")
