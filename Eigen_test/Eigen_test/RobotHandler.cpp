//
//  RobotHandler.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 3.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#include "RobotHandler.hpp"



// TEMP HERE
#include "DumpbedLeastSquares.hpp"
#include "JacobianPseudoInverse.hpp"
#include "JacobianTranspose.hpp"

RobotHandler::RobotHandler(std::shared_ptr<Robot> robot, std::shared_ptr<AbstractSolver> solver):
    robot(robot),
    solver(solver) {
}

RobotHandler::~RobotHandler() {
}

Eigen::Matrix4f RobotHandler::forwardKinematics() {
    return robot->giveMeFullHM();
}

Eigen::VectorXf RobotHandler::calculateInverseKinematics() {
    return solver->calculateData();
}

void RobotHandler::setSolver(std::shared_ptr<AbstractSolver> solver) {
    this->solver = solver;
}

void RobotHandler::setDesiredPosistion(Eigen::VectorXf &desired_position) {
    this->solver->setDesiredPosistion(desired_position);
}
 
