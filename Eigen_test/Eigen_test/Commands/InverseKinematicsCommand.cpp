//
//  InverseKinematics.cpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#include "InverseKinematicsCommand.hpp"
#include <iostream>
#include <Eigen/Dense>
#include "AbstractSolver.hpp"

using namespace Eigen;

InverseKinematicsCommand::InverseKinematicsCommand(std::shared_ptr<AbstractSolver> solver):
    solver(solver) {
}

void InverseKinematicsCommand::execute() {
    VectorXf desired_position(6);
    float posX, posY, posZ;
    std::cout<< "Enter position (X, Y, Z): ";
    std::cin >> posX >> posY >> posZ;
    std::cout<< "Calculating ... \n";
    desired_position << posX, posY, posZ, 0.0, 0.0, 0.0;

    solver->setDesiredPosistion(desired_position);
    solver->calculateData();
}

std::string InverseKinematicsCommand::key() {
    return "inverse_kinematics_command";
}
