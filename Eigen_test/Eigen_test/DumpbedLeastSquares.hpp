//
//  DumpbedLeastSquares.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef DumpbedLeastSquares_hpp
#define DumpbedLeastSquares_hpp

#include <stdio.h>
#include "AbstractSolver.hpp"
#include "MatrixFactory.hpp"
#include "Robot.hpp"

class DumpedLeastSquares: public AbstractSolver {
    MatrixFactory* mtxinstance;
    Eigen::VectorXf& _desired_position;
    Robot& _robot;
    Eigen::VectorXf current_position;
    float nu;

public:
    DumpedLeastSquares(Eigen::VectorXf& desired_position, Robot& robot);
    ~DumpedLeastSquares() { }
    Eigen::VectorXf calculateData();
    void setAdditionalParameter(float& add_in) { nu = add_in;}
    void updateJoints(Eigen::VectorXf& delta_theta);
};


#endif /* DumpbedLeastSquares_hpp */
