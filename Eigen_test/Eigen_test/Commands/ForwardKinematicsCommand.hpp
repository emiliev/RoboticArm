//
//  ForwardKinematicsCommand.hpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef ForwardKinematicsCommand_hpp
#define ForwardKinematicsCommand_hpp

#include <stdio.h>
#include "Command.hpp"
#include "Robot.hpp"

class ForwardKinematicsCommand: public Command {
    std::shared_ptr<Robot> robot;
public:
    ForwardKinematicsCommand(std::shared_ptr<Robot> robot);
    void execute();
    std::string key();
};

#endif /* ForwardKinematicsCommand_hpp */
