#pragma once
#include <string>

std::string IMUDeviceName = "/dev/ttyS4";
int IMUBaudrate = 115200;

std::string RCSbusDeviceName = "/dev/ttyS6";

std::string CanBusName = "can0";
unsigned char LeftMotorAddress = 4;
unsigned char RightMotorAddress = 3;

int BuzzerPin = 34;
bool BuzzerActiveHigh = true;

//motor 2
int LeftMotorDirPin = 36;
int LeftMotorPWMPin = 39;
//motor 1
int RightMotorDirPin = 40;
int RightMotorPWMPin = 42;

int PinIn =1;
int PinOut =2;