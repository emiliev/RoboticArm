//
//  CalculationManager.cpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#include "CalculationManager.hpp"
#include "CalculationObserver.hpp"

void CalculationManager::notify(degreesVector updates) {
    for(ObserversVector::iterator it = observers.begin(); it != observers.end(); ++it) {
        (*it)->notify(updates);
    }
}

void CalculationManager::addObserver(Observer& observer) {
    observers.push_back(observer);
}

CalculationManager& CalculationManager::instance() {
    static CalculationManager instance;
    return instance;
}

CalculationManager::CalculationManager() {
}
