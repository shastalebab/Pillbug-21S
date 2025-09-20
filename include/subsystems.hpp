#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/distance.hpp"
#include "pros/motors.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Optical colorSens(19);
inline pros::Optical proximitySens(20);
inline pros::Distance distanceSens(1);

inline pros::Motor intakeNone(21);
inline pros::Motor intakeFirst(18);
inline pros::Motor intakeSecond(-8);
inline pros::Motor intakeIndexer(10);
inline ez::Piston scraper('A');
inline ez::Piston descore('B');
inline ez::Piston redirect('C');
inline ez::Piston park('D');

class Jammable {
   private:
	int clock = 0;

   public:
	pros::Motor* motor;
	int* target;
	int limit;
	int attempts;
	float maxTemp;
	bool ignoreSort;
	bool pause;

	bool lock;
	void checkJam();

	Jammable() {
		motor = &intakeFirst;
		target = nullptr;
		attempts = 20;
		limit = 4;
		maxTemp = 55;
		ignoreSort = true;
		pause = false;
		lock = false;
	}
	Jammable(pros::Motor* Motor, int* Target, int Limit, int Attempts, float MaxTemp, bool IgnoreSort, bool Pause) {
		motor = Motor;
		target = Target;
		attempts = Attempts;
		limit = Limit;
		maxTemp = MaxTemp;
		ignoreSort = IgnoreSort;
		pause = Pause;
		lock = false;
	}
};

enum Colors { BLUE = 0, NEUTRAL = 1, RED = 2 };

extern Colors allianceColor;
extern Jammable none;
extern Jammable first;
extern Jammable second;
extern Jammable indexer;

extern Jammable* targetMotor;

bool shift();

void setIntake(int first_speed, int second_speed, int third_speed, bool redirect_up);
void setIntake(int first_speed, int second_speed, int third_speed);
void setIntake(int intake_speed, int outtake_speed, bool redirect_up);
void setIntake(int intake_speed, int outtake_speed);
void setIntake(int speed, bool redirect_up);
void setIntake(int speed);
void setScraper(bool state);
void setDescore(bool state);
void setPark(bool state);

void setAlliance(Colors alliance);
void colorToggle();
void colorSet(Colors color, lv_obj_t* object);

void sendHaptic(string input);

void setIntakeOp();
void setScraperOp();
void setDescoreOp();
void setParkOp();

void colorTask();
void antiJamTask();
void controllerTask();