//
//  ForwardKinematicsCommand.cpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#include "ForwardKinematicsCommand.hpp"
#include <iostream>

ForwardKinematicsCommand::ForwardKinematicsCommand(std::shared_ptr<Robot> robot):
    robot(robot) {
}

void ForwardKinematicsCommand::execute() {
    auto numJoints = robot->giveMeJoints().size();
    std::vector<float> degrees;
    for(int index = 0; index < numJoints; ++index) {
        std::cout << "Enter angle: ";
        float degree;
        std::cin >> degree;
        degrees.push_back(degree);
    }
    robot->rotateJoints(degrees);
    robot->giveMeFullHM();
}

std::string ForwardKinematicsCommand::key() {
    return "forward_kinemastics_command";
}
