/**
 * BICOPTER with altitude control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "state/nicla/NiclaConfig.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/SensorSuite.h"
#include <Arduino.h>
#include <ESP32Servo.h>

// Robot
Robot* myRobot = nullptr;

// Communication
BaseCommunicator* baseComm = nullptr;

// Control input from base station
ControlInput behave;
ControlInput cmd;
ReceivedData rcv;

// Data storage for the sensors
float senses[16]; // Adjust size as needed

// Constants and variables
const int TIME_STEP_MICRO = 4000;
int dt = 1000;
unsigned long clockTime;
unsigned long printTime;
int niclaOffset = 11;

// Terms for Nicla controller
nicla_t terms;

void setup() {
    Serial.begin(115200);
    Serial.println("Start!");
    clockTime = micros();
    printTime = micros();
    
    // Initialize communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // Initialize robot with new parameters
    myRobot = RobotFactory::createRobot("FullBicopter");
    
    // Update parameters
    paramUpdate();

    // Update the ground altitude for the ground feedback
    // TODO: make some way to access the actual ground height from robot
    int numSenses = myRobot->sense(senses);
}

void loop() {
    // Retrieve command parameters from ground station and check flags
    recieveCommands();

    // Get sensor values
    int numSenses = myRobot->sense(senses);

    // Send values to ground station
    rcv.flag = 1;
    for (int i = 0; i < numSenses; i++) {
        rcv.values[i] = senses[i];
    }
    bool sent = baseComm->sendMeasurements(&rcv);

    // Print sensor values every second
    if (micros() - printTime > 1000000) { // 1 second
        for (int i = 0; i < numSenses - 1; i++) {
            Serial.print(senses[i]);
            Serial.print(",");
        }
        Serial.println(senses[numSenses - 1]);
        printTime = micros();
    }

    // Nicla controller (when the incoming flag = 2)
    if (cmd.params[0] == 2) {
        int nicla_flag = (int)senses[niclaOffset + 0];
        if (nicla_flag != 0) {
            float _yaw = senses[5];  
            float _height = senses[1];  
            float tracking_x = (float)senses[niclaOffset + 1];
            float tracking_y = (float)senses[niclaOffset + 2];
            
            float x_cal = tracking_x / terms.n_max_x;
            float des_yaw = ((x_cal - 0.5)) * terms.x_strength;
            float y_cal = tracking_y / terms.n_max_y;
            if ( abs(x_cal - 0.5) < .16 && terms.y_strength != 0) {
                z_estimator =  (_height + terms.y_strength * (y_cal - terms.y_thresh));
            }
        } 

        behave.params[0] = cmd.params[0]; // flag
        behave.params[1] = cmd.params[1]; // fx ('meters'/second)
        behave.params[2] = cmd.params[2]; // fz (meters)
        behave.params[3] = 0; // tx (radians/second)
        behave.params[4] = nicla_yaw; // tz (radians)

    } else { // Direct control with joystick if 'flag' is not 2
        z_estimator = cmd.params[2];
        nicla_yaw = cmd.params[4]; // Autoset for when switch occurs
        behave.params[0] = cmd.params[0]; // Flag
        behave.params[1] = cmd.params[1]; // fx
        behave.params[2] = cmd.params[2]; // fz
        behave.params[3] = cmd.params[3]; // tx
        behave.params[4] = cmd.params[4]; // tz
    }

    // Send command to the actuators
    myRobot->control(senses, behave.params, 5);

    // Make the clock rate of the loop consistent
    fixClockRate();
}

void recieveCommands() {
    if (baseComm->isNewMsgCmd()) {
        // New command received
        cmd = baseComm->receiveMsgCmd();
        if (int(cmd.params[11]) == 1) {
            paramUpdate();
        }
        // Print command
        Serial.print("Cmd arrived: ");
        printControlInput(cmd);
    }
}

void paramUpdate() {
    NiclaConfig::getInstance()->loadConfiguration();
    const nicla_t& config = NiclaConfig::getInstance()->getConfiguration();
    terms = config; // Copy configuration data
    myRobot->getPreferences();
    baseComm->setMainBaseStation();
}

void fixClockRate() {
    dt = (int)(micros() - clockTime);
    while (TIME_STEP_MICRO - dt > 0) {
        dt = (int)(micros() - clockTime);
    }
    clockTime = micros();
}
