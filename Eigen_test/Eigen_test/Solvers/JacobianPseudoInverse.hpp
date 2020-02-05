//
//  JacobianPseudoInverse.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 19.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef JacobianPseudoInverse_hpp
#define JacobianPseudoInverse_hpp

#include <stdio.h>
#include "AbstractSolver.hpp"
#include "Robot.hpp"

class JacobianPseudoInverse : public AbstractSolver {
    MatrixFactory* mtxinstance;
    Eigen::VectorXf _desired_position;
    std::shared_ptr<Robot> _robot;
    Eigen::VectorXf current_position;

public:
    JacobianPseudoInverse(std::shared_ptr<Robot> robot);
    void setDesiredPosistion(Eigen::VectorXf& desired_position);
    Eigen::VectorXf calculateData();
    void setAdditionalParameter(float& add_in){ }
    void updateJoints(Eigen::VectorXf& delta_theta);
};

#endif /* JacobianPseudoInverse_hpp */
