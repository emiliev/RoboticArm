//
//  SolverBuilder.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 3.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef SolverBuilder_hpp
#define SolverBuilder_hpp

#include <stdio.h>
#include "JacobianTranspose.hpp"
#include "JacobianPseudoInverse.hpp"
#include "Robot.hpp"
class SolverBuilder {
public:
    static std::shared_ptr<AbstractSolver> createSolver(char symbol, std::shared_ptr<Robot> robot) {
        std::shared_ptr<AbstractSolver> solver;
        if (symbol == 'T' || symbol == 't') {
            solver = std::make_shared<JacobianTranspose>(robot);
        } else {
            solver = std::make_shared<JacobianPseudoInverse>(robot);
        }
        return solver;
    }
};

//0.0197859
//120.64
//-86.7482
//179.441
//-90.4128

//0.000560046
//56.5898
//-64.4274
//179.998
//-136.472
#endif /* SolverBuilder_hpp */
