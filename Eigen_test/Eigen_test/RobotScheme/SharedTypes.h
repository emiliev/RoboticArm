//
//  SharedTypes.h
//  Eigen_test
//
//  Created by Emil Iliev on 18.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef SharedTypes_h
#define SharedTypes_h

#include <vector>
#include <map>
#include <string>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#define DHINDEX(x) x-1
#define FMACRO(axis_one , axis_two) (axis_one + nu*axis_two)/(1 + nu)

enum JointT {REVOLUTE , PRISMATIC , CONSTANTJOINT , NOTSET};
enum AxisT  {AxisX , AxisY , AxisZ};

struct dh_parametrs {
    float a;            //Length of common normal
    float alpha;        //Angle between zi and zi-1 along xi
    float d;            //distance between xi and xi-1 along zi (variable for prismatic)
    float theta;        //Angle between xi and xi-1 along zi    (variavle for revolute)
    JointT z_joint_type;//Joint type at z-1 axis
    std::string joint_name;
    
    dh_parametrs(float length, float alpha_, float prismatic_distance, float theta_, JointT jointType, std::string name):
        a(length),
        alpha(alpha_),
        d(prismatic_distance),
        theta(theta_),
        z_joint_type(jointType),
        joint_name(name) {
    }
};

typedef std::vector<dh_parametrs> dh_table;
typedef std::vector<Eigen::Vector3f , Eigen::aligned_allocator<Eigen::Vector3f> > vector_storage;
typedef std::vector<Eigen::Matrix4f , Eigen::aligned_allocator<Eigen::Matrix4f> > HomMatrixHolder;

#endif /* SharedTypes_h */
