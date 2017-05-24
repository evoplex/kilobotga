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

#ifndef PD_CTRL_H
#define PD_CTRL_H

#include "abstractga_ctrl.h"

/**
 * @brief The PDCtrl class.
 * Prisoner's Dilemma game
 * @author Marcos Cardinot <mcardinot@gmail.com>
 */
class PDCtrl : public AbstractGACtrl
{

public:
    PDCtrl();
    virtual ~PDCtrl() {}

    // generate a random gene (pure game strategy)
    virtual QVariant randGene() const;

    // set chromosome (vector of motor speeds)
    virtual bool setChromosome(Chromosome chromosome);

    // CCI_Controller stuff
    virtual void Init(TConfigurationNode &t_node);
    virtual void Reset();
    virtual void ControlStep();

private:
    CRandom::CRNG* m_pcRNG; // random number generator
    message_t m_message;

    uint8_t m_curStrategy;
    CColor m_curColor;

    float calcPerformance(uint8_t sA, uint8_t sB);
};

#endif // PD_CTRL_H
