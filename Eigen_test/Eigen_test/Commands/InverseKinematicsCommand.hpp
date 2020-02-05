//
//  InverseKinematicsCommand_hpp.hpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef InverseKinematicsCommand_hpp
#define InverseKinematicsCommand_hpp
#include <memory>
#include <stdio.h>
#include "Command.hpp"
class AbstractSolver;

class InverseKinematicsCommand: public Command {
public:
    InverseKinematicsCommand(std::shared_ptr<AbstractSolver> solver);
    void execute();
    std::string key();
private:
    std::shared_ptr<AbstractSolver> solver;
};

#endif /* InverseKinematicsCommand_hpp */
