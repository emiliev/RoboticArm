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

using namespace Eigen;

using namespace std;

int main(int argc, const char * argv[]) {
    
    dh_table table;
//    dh_parametrs joint1;
    //base revolute
                            // a,    alpha, d,  theta
//    dh_parametrs base_joint(0,      0,  60,   0, JointT::REVOLUTE, "Base Joint");
//    dh_parametrs joint1(    14,     90, 36,    90, JointT::REVOLUTE, "Joint1");
//    dh_parametrs joint2(    120,    0,  0,    90, JointT::REVOLUTE, "Joint2");
//    dh_parametrs joint3(    120,    90, 0,    0, JointT::REVOLUTE, "Joint3");
//    dh_parametrs joint4(    60,      0, 0,   0, JointT::REVOLUTE, "Joint4");


/* TESTING */
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
    
    
//    float thita0 = 0;
//    float thita1 = 122.5;
//    float thita2 = -90;
//    float thita3 = 180;
//    float thita4 = -90;
//
//    float alpha0 = 90;
//    float alpha1 = 0;
//    float alpha2 = -90;
//    float alpha3 = -90;
//    float alpha4 = 0;

    dh_parametrs base_joint(14,  alpha0, 97.0,   thita0, JointT::REVOLUTE, "Base Joint");
    dh_parametrs joint1(    120, alpha1, 0,      thita1, JointT::REVOLUTE, "Joint1");
    dh_parametrs joint2(    5.3, alpha2, -11.5,   thita2, JointT::REVOLUTE, "Joint2");
    dh_parametrs joint3(    5.1, alpha3, 117.7, thita3, JointT::REVOLUTE, "Joint3");
    dh_parametrs joint4(    62.5, alpha4, -3.4,  thita4, JointT::REVOLUTE, "Joint4");

    /*  TESTING FIRST JOINT */
//    table.push_back(base_joint);
    
    /* TESTING SECOND JOINT */
//    table.push_back(base_joint);
//    table.push_back(joint1);

    /* TESTING THIRD JOINT */
//    table.push_back(base_joint);
//    table.push_back(joint1);
//    table.push_back(joint2);

    /* TESTING FOURTH JOIN */
//    table.push_back(base_joint);
//    table.push_back(joint1);
//    table.push_back(joint2);
//    table.push_back(joint3);

    /* TESTING 5TH JOIN */
    table.push_back(base_joint);
    table.push_back(joint1);
    table.push_back(joint2);
    table.push_back(joint3);
    table.push_back(joint4);
    
//    dh_parametrs base_joint(0,  0,      86,     30, JointT::REVOLUTE, "Base Joint");
//    dh_parametrs sholder(   0,  90, 0,      50, JointT::REVOLUTE, "Joint1");
//    dh_parametrs elbow(     96, 0,      0,      45, JointT::REVOLUTE, "Joint2");
//    dh_parametrs wrist(     96, 0,      0,      25, JointT::REVOLUTE, "Joint3");
//    dh_parametrs gripper(   0,  90, 59.5,   0, JointT::REVOLUTE, "Joint4");
//    table.push_back(base_joint);
//    table.push_back(sholder);
//    table.push_back(elbow);
//    table.push_back(wrist);
//    table.push_back(gripper);

    auto origin = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Robot robot(origin);
    robot.loadConfig(table);
    
    
    
    
//    while(true) {
//        char symbol;
//        std::cin >> symbol;
//        switch (symbol) {
//
//        /// Forward kinematics
//        case 'f':
//        case 'F':
//            robot.giveMeFullHM();
//            break;
//
//        /// Inverse kinematics
//        case 'i':
//        case 'I':
//
//            break;
//
//        }
//    }
//
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
//    Forward kinematics
    robot.printFullTransformationMatrix();
//    robot.printHomogenTransformationMatrix();

    
    // Inverse kinematics
    VectorXf des(6);
    des << -147.129 , 14.9 , 350.294 , 0.0f , 0.0f , 0.0f;
//    des << 60.0f, 0.0f, 330.0f, 0.0f, 0.0f, 0.0f;
//    des << 76.0f, 88.0f, 244.0f, 0.0f , 0.0f , 0.0f;

//    AbstractSolver* pJpt = new JacobianPseudoInverse(robot);
//    AbstractSolver* pJpt = new JacobianTranspose(des, robot);
    
    
//    AbstractSolver* pJpt = new DumpedLeastSquares(des, robot);
//    float speccfc = 1.0f;
//    pJpt->setAdditionalParameter(speccfc);
    
    
//    pJpt->calculateData(des);
//    robot.printConfiguration();
//    delete pJpt;
    
    return 0;
}
