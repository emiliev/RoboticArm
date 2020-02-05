//
//  CommandBuilder.cpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#include "CommandBuilder.hpp"
#include "AbstractSolver.hpp"
#include "Robot.hpp"
#include "ForwardKinematicsCommand.hpp"
#include "InverseKinematicsCommand.hpp"
#include <memory>

std::shared_ptr<CommandManager> CommandManagerBuilder::createManager(std::shared_ptr<Robot> currentRobot, std::shared_ptr<AbstractSolver> currentSolver) {
    std::shared_ptr<CommandManager> manager = std::make_shared<CommandManager>();
    std::shared_ptr<ForwardKinematicsCommand> forwardKinematicsCommand = std::make_shared<ForwardKinematicsCommand>(currentRobot);
    std::shared_ptr<InverseKinematicsCommand> inverseKinematicsCommand = std::make_shared<InverseKinematicsCommand>(currentSolver);
    manager->addCommand(forwardKinematicsCommand);
    manager->addCommand(inverseKinematicsCommand);
    return manager;
}

