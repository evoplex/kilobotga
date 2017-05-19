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

#ifndef DEMO_CTRL_H
#define DEMO_CTRL_H

#include "abstractga_ctrl.h"

/**
 * @brief The DemoCtrl class
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class DemoCtrl : public AbstractGACtrl
{

public:
    DemoCtrl();
    virtual ~DemoCtrl() {}

    // generate a random gene (motor speed)
    virtual Gene randGene() const;

    // set chromosome (vector of motor speeds)
    virtual bool setChromosome(Chromosome chromosome);

    // CCI_Controller stuff
    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();
    virtual void Reset();

private:
     CRandom::CRNG*  m_pcRNG; // random number generator
     size_t m_iLUTSize; // lookup table size; it'll define the chromossome size
     std::vector<uint8_t> m_lutDistance; // range distances for each gene

     // initialize our lookup tables with random values
     void initLUT();

     // get a lut index from a distance (in mm)
     size_t getLUTIndex(uint8_t distance) const;

     // calculate the local performance
     // power-law: a*(x+1)^b.
     float calcPerformance(uint8_t distance) const;
};

#endif // DEMO_CTRL_H
