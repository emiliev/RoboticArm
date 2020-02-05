//
//  CalculationManager.hpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef CalculationManager_hpp
#define CalculationManager_hpp

#include <stdio.h>

#include <memory>
#include <vector>
class CalculationObserver;
typedef std::shared_ptr<CalculationObserver> Observer;
typedef std::vector<Observer> ObserversVector;
typedef std::vector<float> degreesVector;

class CalculationManager {
public:
    
    static CalculationManager& instance();
    void addObserver(Observer& observer);
    void notify(degreesVector updates);

private:
    ObserversVector observers;
    CalculationManager();

};

#endif /* CalculationObservable_hpp */
