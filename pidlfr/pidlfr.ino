#include <PID_v1_bc.h>

class LineFollowerRobot {
private:
    // Motor
    int TT = 6;
    int NT = 3;
    int TP = 10;
    int NP = 11;
    // Sensor
    int Left = A0;
    int M_Left = A1;
    int Middle = A2;
    int M_Right = A3;
    int Right = A4;
    //
    int outline = 0;
    int VLeft;
    int VM_Left;
    int VMiddle;
    int VM_Right;
    int VRight;
    int thamchieu = 350;
    int leftsp;
    int rightsp;
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
        pinMode(M_Left, INPUT);
        pinMode(Middle, INPUT);
        pinMode(M_Right, INPUT);
        pinMode(Right, INPUT);
        pinMode(TT, OUTPUT);
        pinMode(NT, OUTPUT);
        pinMode(TP, OUTPUT);
        pinMode(NP, OUTPUT);
        Setpoint = 0;
        myPID.SetMode(AUTOMATIC);
        myPID.SetSampleTime(1);
        myPID.SetOutputLimits(-speedposito / 3, speedposito / 3);
    }

    int convert(int a, int b) {
        return (a > b) ? 0 : 1;
    }

    void readSensor() {
        VLeft = convert(analogRead(Left), thamchieu);
        VM_Left = convert(analogRead(M_Left), thamchieu);
        VMiddle = convert(analogRead(Middle), thamchieu);
        VM_Right = convert(analogRead(M_Right), thamchieu);
        VRight = convert(analogRead(Right), thamchieu);
    }

    int error() {
        if (VLeft == 1 && VM_Left == 1 && VMiddle == 1 && VM_Right == 1 && VRight == 1) {
            return (outline > 0) ? 10 : -10;
        } else {
            int result = ((VLeft * (-4)) + (VM_Left * (-2)) + (VMiddle * 0) + (VM_Right * 2) + (VRight * 4)) / (5 - (VLeft + VM_Left + VMiddle + VM_Right + VRight));
            return result;
        }
    }

    void robotRun() {
        Input = error();
        outline = Input;
        if (Input == 0) {
            analogWrite(TT, speedposito);
            analogWrite(TP, speedposito);
        } else if (Input == 10 || Input == -10) {
            if (Input == 10) {
                analogWrite(TT, 0);
                analogWrite(NP, 0);
                analogWrite(NT, speedposito / 2);
                analogWrite(TP, speedposito);
            } else {
                analogWrite(NT, 0);
                analogWrite(TP, 0);
                analogWrite(TT, speedposito);
                analogWrite(NP, speedposito / 2);
            }
        } else {
            myPID.Compute();
            leftsp = speedposito + Output;
            rightsp = speedposito - Output;
            leftsp = constrain(leftsp, 0, 255);
            rightsp = constrain(rightsp, 0, 255);
            analogWrite(TT, leftsp);
            analogWrite(NT, 0);
            analogWrite(TP, rightsp);
            analogWrite(NP, 0);
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
