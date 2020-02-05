//
//  JacobianTranspose.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 19.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef JacobianTranspose_hpp
#define JacobianTranspose_hpp

#include <stdio.h>
#include "AbstractSolver.hpp"
#include "MatrixFactory.hpp"
#include "Robot.hpp"

class JacobianTranspose : public AbstractSolver
{
    MatrixFactory*  mtxinstance;
    float lamda_coefficent;
    Eigen::VectorXf _desired_position;
    Eigen::VectorXf current_position;
    std::shared_ptr<Robot> _robot;

public:
    void setDesiredPosistion(Eigen::VectorXf&  desired_position);
    JacobianTranspose(std::shared_ptr<Robot> robot);
    Eigen::VectorXf calculateData();
    void setAdditionalParameter(float& add_in);
    void updateJoints(Eigen::VectorXf& delta_theta);
};

#endif /* JacobianTranspose_hpp */
