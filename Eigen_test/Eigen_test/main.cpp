//
//  main.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#include <iostream>
#include <Eigen/Dense>
#include "SharedTypes.h"
#include <math.h>
#include "Robot.hpp"
#include "DumpbedLeastSquares.hpp"
#include "JacobianPseudoInverse.hpp"
#include "JacobianTranspose.hpp"
#include "RobotHandler.hpp"
#include "SolverBuilder.hpp"
using namespace Eigen;

using namespace std;

int main(int argc, const char * argv[]) {

    dh_table table;
    float thita0 = 0;
    float thita1 = 90;
    float thita2 = -90;
    float thita3 = 180;
    float thita4 = -90;

    float alpha0 = 90;
    float alpha1 = 0;
    float alpha2 = -90;
    float alpha3 = -90;
    float alpha4 = 0;

    dh_parametrs base_joint(14,  alpha0, 97.0,   thita0, JointT::REVOLUTE, "Base Joint");
    dh_parametrs joint1(    120, alpha1, 0,      thita1, JointT::REVOLUTE, "Joint1");
    dh_parametrs joint2(    5.3, alpha2, -11.5,   thita2, JointT::REVOLUTE, "Joint2");
    dh_parametrs joint3(    5.1, alpha3, 117.7, thita3, JointT::REVOLUTE, "Joint3");
    dh_parametrs joint4(    62.5, alpha4, -3.4,  thita4, JointT::REVOLUTE, "Joint4");
    table.push_back(base_joint);
    table.push_back(joint1);
    table.push_back(joint2);
    table.push_back(joint3);
    table.push_back(joint4);

    auto origin = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    std::shared_ptr<Robot> robot = std::make_shared<Robot>(origin);
    robot->loadConfig(table);

    auto solver = SolverBuilder::createSolver('P', robot);
    RobotHandler handler(robot, solver);
    
    robot->printFullTransformationMatrix();
    bool isRunning = true;
    while(isRunning) {
        char symbol;
        std::cout << "Enter symbol: ";
        std::cin >> symbol;
        switch (symbol) {
        /// Forward kinematics
        case 'f':
        case 'F':
            {
                handler.forwardKinematics();
                robot->printFullTransformationMatrix();
                break;
            }
        /// Inverse kinematics
        case 'i':
        case 'I':
            {
                VectorXf des(6);
                float posX, posY, posZ;
                std::cout<< "Enter position (X, Y, Z): ";
                std::cin >> posX >> posY >> posZ;
                std::cout<< "Calculating ... \n";
                des << posX, posY, posZ, 0.0, 0.0, 0.0;
                //  -147.129 , 14.9 , 350.294
                handler.setDesiredPosistion(des);
                handler.calculateInverseKinematics();
                robot->printConfiguration();
                break;
            }
        case 'q' :
        case 'Q' :
            {
                isRunning = false;
                break;
            }
        }
    }

    return 0;
}
