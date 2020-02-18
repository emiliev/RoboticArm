//
//  main.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#include <iostream>
//#include <fstream>
#include <Eigen/Dense>
#include "SharedTypes.h"
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include "Robot.hpp"
#include "SolverBuilder.hpp"
#include "CommandBuilder.hpp"

using namespace Eigen;
using namespace std;

///TODO: Move to Another file and rename to SerialCommunicator ?
#include "CalculationObserver.hpp"
#include "CalculationManager.hpp"
//#include <serial.h>
//#include <sstream>
class SerialLogger: public CalculationObserver {
public:
    void notify(std::vector<float> updates) {
        int outputFD = open("/dev/cu.usbserial-1420", O_RDWR);
        if (outputFD == -1) {
            fprintf(stderr, "Stream was unable to open!\n");
            return;
        }

        stringstream outputStream;
        for (std::vector<float>::iterator it = updates.begin(); it != updates.end(); ++it) {
            outputStream << (*it) << " ";
        }
        std::string outputData = outputStream.str();
        size_t size = outputData.size();
        write(outputFD, outputData.c_str(), size);
        char buf[16];
//        read(outputFD, buf, 16);
//        puts(buf);
        close(outputFD);
    }
};


class ConsoleLogger: public CalculationObserver {
public:
    void notify(std::vector<float> updates) {
        for (std::vector<float>::iterator it = updates.begin(); it != updates.end(); ++it) {
            std::cout << (*it) << "\n";
        }
    }
};


void printMenu() {
    cout << "\t Menu \t \n" <<
        "i(I) - Inverse Kinematics\n" <<
        "f(F) - Forward Kinematics\n" <<
        "q(Q) - Quit\n" <<
        "h(H) - Show menu\n";
}

/*
std::shared_ptr<Robot> createRobot() {
    float thita0 = 0;
    float thita1 = 90;
    float thita2 = -90;
    float thita3 = 180;
    float thita4 = -90;

    float alpha0 = 90;
    float alpha1 = 0;
    float alpha2 = -90;
    float alpha3 = -90;
    float alpha4 = 0;

    dh_parametrs base_joint(14,  alpha0, 97.0,   thita0, JointT::REVOLUTE, "Base Joint");
    dh_parametrs joint1(    120, alpha1, 0,      thita1, JointT::REVOLUTE, "Joint1");
    dh_parametrs joint2(    5.3, alpha2, -11.5,   thita2, JointT::REVOLUTE, "Joint2");
    dh_parametrs joint3(    5.1, alpha3, 117.7, thita3, JointT::REVOLUTE, "Joint3");
    dh_parametrs joint4(    62.5, alpha4, -3.4,  thita4, JointT::REVOLUTE, "Joint4");
    dh_table table;
    table.push_back(base_joint);
    table.push_back(joint1);
    table.push_back(joint2);
    table.push_back(joint3);
    table.push_back(joint4);

    auto origin = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    std::shared_ptr<Robot> robot = std::make_shared<Robot>(origin);
    robot->loadConfig(table);
    return robot;
}

int main(int argc, const char * argv[]) {

    std::shared_ptr<Robot> robot = createRobot();
    cout << "Choose inverse kinematics method, JacobianPseudoinverse(P) or JacobianTranspose(T): ";
    char methodSymbol;
    cin >> methodSymbol;
    auto solver = SolverBuilder::createSolver(methodSymbol, robot);

    robot->printFullTransformationMatrix();
    std::shared_ptr<CommandManager> manager = CommandManagerBuilder::createManager(robot, solver);
    
    std::shared_ptr<CalculationObserver> testNotifier = std::make_shared<Logger>();
    CalculationManager::instance().addObserver(testNotifier);
    bool isRunning = true;
    while(isRunning) {
        char symbol;
        std::cout << "Enter symbol: ";
        std::cin >> symbol;
        switch (symbol) {
        /// Forward kinematics
        case 'f':
        case 'F':
            {
                manager->executeCommand("forward_kinemastics_command");
                break;
            }
        /// Inverse kinematics
        case 'i':
        case 'I':
            {
                manager->executeCommand("inverse_kinematics_command");
                // -147.129 14.9 350.294
                break;
            }
        case 'h':
        case 'H':
            {
                printMenu();
                break;
            }
        case 'q' :
        case 'Q' :
            {
                isRunning = false;
                break;
            }
        }
    }

    return 0;
}*/

int main(int argc, const char * argv[]) {
    std::vector<float> degress;
    degress.push_back(10);
    degress.push_back(10);
    degress.push_back(10);
    degress.push_back(10);
    degress.push_back(10);
    
    ConsoleLogger consoleLogger;
    consoleLogger.notify(degress);
    
    SerialLogger serialLogger;
    serialLogger.notify(degress);
    return 0;
}
