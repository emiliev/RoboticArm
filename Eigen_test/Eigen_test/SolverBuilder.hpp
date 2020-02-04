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
        if (symbol == 'T') {
            solver = std::make_shared<JacobianTranspose>(robot);
        } else {
            solver = std::make_shared<JacobianPseudoInverse>(robot);
        }
        return solver;
    }
};


#endif /* SolverBuilder_hpp */
