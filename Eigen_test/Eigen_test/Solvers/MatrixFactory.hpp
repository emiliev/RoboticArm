//
//  MatrixFactory.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef MatrixFactory_hpp
#define MatrixFactory_hpp

#include <stdio.h>
#include <Eigen/Dense>
#define PI 3.14159265
#define CONVERT_TO_RAD(x) x*(M_PI/180)

class MatrixFactory {
    MatrixFactory() {}

    static MatrixFactory* _instance;
public:

/***********************************************************************
    Rotation matrix
***********************************************************************/

    Eigen::Matrix4f createRotationMatrixAroundZ(float alpha );
    Eigen::Matrix4f createRotationMatrixAroundY(float beta );
    Eigen::Matrix4f createRotationMatrixAroundX(float gamma );


/************************************************************************/
/* Translation matrix                                                   */
/************************************************************************/
    
    Eigen::Matrix4f createTranslateMatrixX(float dx );
    Eigen::Matrix4f createTranslateMatrixY(float dy );
    Eigen::Matrix4f createTranslateMatrixZ(float dz );

/************************************************************************/
/* Transformation matrix from frame i to i-1                            */
/************************************************************************/

    Eigen::Matrix4f calculateHTranslationMatrix(float alpha, float a, float d, float theta);

    /************************************************************************/
/* Help functions                                                       */
/************************************************************************/
    Eigen::Matrix3f extractRotationMatrix(Eigen::Matrix4f & hm );
    Eigen::Vector3f extractTranslationVector (Eigen::Matrix4f & hm );
    Eigen::Vector3f multiplyVectors(Eigen::Vector3f& vec_one ,Eigen::Vector3f& vec_two);
    float getLengthOfVector(Eigen::VectorXf& invec);
/************************************************************************/
/* Full calculation algo                                                */
/************************************************************************/

    static MatrixFactory* getInstance();
};

#endif /* MatrixFactory_hpp */
