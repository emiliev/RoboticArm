//
//  Link.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef Link_hpp
#define Link_hpp

#include <stdio.h>
#include "SharedTypes.h"

class Link {
    Eigen::Vector3f global_link_origin;

    /************************************************************************/
    /* DH parametrs                                                         */
    /************************************************************************/
    float a;
    float alpha;

    //TODO : add mass center

public:
    Link(float a_in , float alpha_in);

    float getCommonNormalParametr_a(){ return a; }
    float getZAxisRotationParametr_aplha(){ return alpha; }
};

#endif /* Link_hpp */
