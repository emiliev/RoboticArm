//
//  AbstractSolver.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef AbstractSolver_hpp
#define AbstractSolver_hpp

#include <stdio.h>
#include <Eigen/Dense>

enum AlgorithmType
{
    JACOBIANTRANSPOSE,
    JACOBIANPSEVDOINVERSE,
    DUMPEDLEASTSQUARES,
    SELECTIVEDUMPEDLEASTSQUARES,
    CCD
};

//Abstarct class for all algorithm
class AbstractSolver
{
public:
             AbstractSolver(){}
    virtual ~AbstractSolver(){}
    virtual Eigen::VectorXf calculateData() = 0;
    virtual void setAdditionalParameter(float& add_in) = 0;
};


#endif /* AbstractSolver_hpp */
