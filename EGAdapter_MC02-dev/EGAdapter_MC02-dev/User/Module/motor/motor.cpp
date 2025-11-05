#include "motor.h"

#include "DJImotor.h"
#include "DMmotor.h"


void Motor::controlLoop() {
	DJIMotor::controlLoop();
	DMMotor::controlLoop();
}
void Motor::enableAll() {
	DJIMotor::enableAll();
	DMMotor::enableAll();
}
void Motor::disableAll() {
	DJIMotor::disableAll();
	DMMotor::disableAll();
}

bool Motor::hasDisabledMotor() {
	return false;
}


std::unique_ptr<Motor> Motor::create(const Motor::Config &config) {
	switch (config.motor_type) {
		case MotorType::GM6020:
		case MotorType::M3508:
		case MotorType::M2006:
			return std::make_unique<DJIMotor>(config);

		case MotorType::DM_MOTOR:
			return std::make_unique<DMMotor>(config);



		case MotorType::OTHER:
		default:
			return nullptr;
	}
}