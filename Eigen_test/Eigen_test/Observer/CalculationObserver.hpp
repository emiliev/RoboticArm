//
//  CalculationObserver.hpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef CalculationObserver_hpp
#define CalculationObserver_hpp

#include <stdio.h>

class CalculationObserver {
public:
    virtual void notify(std::vector<float> updates) = 0;
};

#endif /* CalculationObserver_hpp */
