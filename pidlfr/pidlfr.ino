#include <PID_v1_bc.h>

class LineFollowerRobot {
private:
    // Motor
    int LF = 6; //Left-Front
    int LB = 3; //Left-Back
    int RF = 10; //Right-Front
    int RB = 11; //Right-Back
    // Sensor
    int Left = A0;
    int MidLeft = A1;
    int Middle = A2;
    int MidRight = A3;
    int Right = A4;
    //
    int outline = 0;
    int VarLeft;
    int VarMidLeft;
    int VarMiddle;
    int VarMidRight;
    int VarRight;
    int threshold = 350;
    int leftspeed;
    int rightspeed;
    int speedposito = 100;

    // PID Variables
    double Setpoint, Input, Output;
    int ten = 10;
    double Kp = 10, Ki = 0, Kd = 10;
    PID myPID;

public:
    LineFollowerRobot() : myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT) {}

    void setup() {
        Serial.begin(9600);
        pinMode(Left, INPUT);
        pinMode(MidLeft, INPUT);
        pinMode(Middle, INPUT);
        pinMode(MidRight, INPUT);
        pinMode(Right, INPUT);
        pinMode(LF, OUTPUT);
        pinMode(LB, OUTPUT);
        pinMode(RF, OUTPUT);
        pinMode(RB, OUTPUT);
        Setpoint = 0;
        myPID.SetMode(AUTOMATIC);
        myPID.SetSampleTime(1);
        myPID.SetOutputLimits(-speedposito / 3, speedposito / 3);
    }

    int convert(int a, int b) {
        return (a > b) ? 0 : 1;
    }

    void readSensor() {
        VarLeft = convert(analogRead(Left), threshold);
        VarMidLeft = convert(analogRead(MidLeft), threshold);
        VarMiddle = convert(analogRead(Middle), threshold);
        VarMidRight = convert(analogRead(MidRight), threshold);
        VarRight = convert(analogRead(Right), threshold);
    }

    int error() {
        if (VarLeft == 1 && VarMidLeft == 1 && VarMiddle == 1 && VarMidRight == 1 && VarRight == 1) {
            return (outline > 0) ? 10 : -10;
        } else {
            int result = ((VarLeft * (-4)) + (VarMidLeft * (-2)) + (VarMiddle * 0) + (VarMidRight * 2) + (VarRight * 4)) / (5 - (VarLeft + VarMidLeft + VarMiddle + VarMidRight + VarRight));
            return result;
        }
    }

    void robotRun() {
        Input = error();
        outline = Input;
        if (Input == 0) {
            analogWrite(LF, speedposito);
            analogWrite(LB, speedposito);
        } else if (Input == 10 || Input == -10) {
            if (Input == 10) {
                analogWrite(LF, 0);
                analogWrite(RB, 0);
                analogWrite(LB, speedposito / 2);
                analogWrite(RF, speedposito);
            } else {
                analogWrite(LB, 0);
                analogWrite(RF, 0);
                analogWrite(LF, speedposito);
                analogWrite(RB, speedposito / 2);
            }
        } else {
            myPID.Compute();
            leftspeed = speedposito + Output;
            rightspeed = speedposito - Output;
            leftspeed = constrain(leftspeed, 0, 255);
            rightspeed = constrain(rightspeed, 0, 255);
            analogWrite(LF, leftspeed);
            analogWrite(LB, 0);
            analogWrite(RF, rightspeed);
            analogWrite(RB, 0);
        }
    }

    void loop() {
        readSensor();
        robotRun();
    }
};

LineFollowerRobot myRobot;

void setup() {
    myRobot.setup();
}

void loop() {
    myRobot.loop();
}
