//
//  RobotHandler.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 3.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef RobotHandler_hpp
#define RobotHandler_hpp

#include <Eigen/StdVector>
#include <stdio.h>

#include "Robot.hpp"
#include "AbstractSolver.hpp"

class RobotHandler {
private:
    std::shared_ptr<Robot> robot;
    std::shared_ptr<AbstractSolver> solver;
public:
    RobotHandler(std::shared_ptr<Robot> robot, std::shared_ptr<AbstractSolver> solver);
    ~RobotHandler();

    Eigen::Matrix4f forwardKinematics();
    void setDesiredPosistion(Eigen::VectorXf& desired_position);
    void setSolver(std::shared_ptr<AbstractSolver> solver);
    Eigen::VectorXf calculateInverseKinematics();
};


#endif /* RobotHandler_hpp */
