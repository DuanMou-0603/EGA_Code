//
// Created by An on 2025/10/18.
//
#include "controller.h"
#include "SMCcontroller.h"

#include "PIDcontroller.h"

std::unique_ptr<Controller> Controller::create(Config config) {
	switch (config.type) {
		case Type::PID:
			return std::make_unique<PIDInstance>(config.pid_config);
		case Type::SMC:
			return std::make_unique<SMCInstance>(config.smc_config);
		default:
			return nullptr;
	}
}