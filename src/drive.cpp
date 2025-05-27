#include "main.h"  // IWYU pragma: keep

AutonMode autonMode = AutonMode::DRIVER;
const double width = 12.5;
Coordinate currentPoint = {0, 0, 0};
vector<Coordinate> autonPath = {};

//
// Internal math
//

double getDistance(Coordinate point1, Coordinate point2, drive_directions direction) {
	auto new_direction = direction == rev ? -1 : 1;
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	return ((sqrt((errorX * errorX) + (errorY * errorY))) * new_direction);
}

double getTheta(Coordinate point1, Coordinate point2, drive_directions direction) {
	auto new_direction = direction == rev ? 180 : 0;
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	return ((atan2(errorX, errorY) * 180 / M_PI) + new_direction);
}

Coordinate getArc(Coordinate startpoint, double right, double left, double distance) {
	// Get the coordinate within the reference frame of the robot of the end point
	double radius = (right + left) / (right - left) * (width / 2);
	double theta = ((right - left) / width * distance) + (startpoint.t * M_PI / 180);

	double relative_x = -((-radius * cos(theta) + radius) - (-radius * cos(startpoint.t * M_PI / 180) + radius));
	double relative_y = -((radius * sin(theta)) - (radius * sin(startpoint.t * M_PI / 180)));

	theta *= 180 / M_PI;
	if(theta < 0) theta += 360;
	theta = fmod(theta, 360);

	Coordinate point_relative = {relative_x, relative_y, theta};

	// Translate the point's x and y values by the start point's x and y values
	point_relative.x += startpoint.x;
	point_relative.y += startpoint.y;

	return point_relative;
}

Coordinate getArcFromTheta(Coordinate startpoint, e_swing side, e_angle_behavior behavior, double right, double left, double theta) {
	// Get the coordinate within the reference frame of the robot of the end point
	double radius = (right + left) / (right - left) * (width / 2);
	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;

	double relative_x = -((-radius * cos(theta * M_PI / 180) + radius) - (-radius * cos(startpoint.t * M_PI / 180) + radius));
	double relative_y = -((radius * sin(theta * M_PI / 180)) - (radius * sin(startpoint.t * M_PI / 180)));

	Coordinate point_relative = {relative_x, relative_y, theta};

	// Translate the point's x and y values by the start point's x and y values
	point_relative.x += startpoint.x;
	point_relative.y += startpoint.y;

	return point_relative;
}

std::vector<Coordinate> injectArc(Coordinate startpoint, e_swing side, e_angle_behavior behavior, double main, double opp, double theta, double lookAhead) {
	double left = side == LEFT_SWING ? main : opp;
	double right = side == RIGHT_SWING ? main : opp;

	if(startpoint.t < 0) startpoint.t += 360;

	std::vector<Coordinate> pointsBar;
	double arciter = 0;
	double arcdist = ((side == LEFT_SWING && behavior == ccw) || (side == RIGHT_SWING && behavior == cw)) ? .01 * lookAhead : -(.01 * lookAhead);
	Coordinate newDist = getArc(startpoint, right, left, arciter);

	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;

	while(!(newDist.t > theta - (3.837 * lookAhead) && newDist.t < theta + (3.837 * lookAhead))) {
		arciter += arcdist;
		newDist = getArc(startpoint, right, left, arciter);
		newDist.movement = MovementType::SWING;
        newDist.main = main;
		pointsBar.push_back(newDist);
	}
	return pointsBar;
}

std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead) {
	if(coordList.size() > 1) {
		std::vector<Coordinate> injectedList = {};
		for(int i = 0; i < coordList.size() - 1; i++) {
			if(coordList[i + 1].movement == MovementType::SWING) {
				std::vector<Coordinate> swingList = injectArc(coordList[i], coordList[i + 1].side, coordList[i + 1].behavior, coordList[i + 1].main,
															  coordList[i + 1].opp, coordList[i + 1].t, lookAhead);
				injectedList.insert(injectedList.end(), swingList.begin(), swingList.end());
			} else if(coordList[i + 1].movement == MovementType::DRIVE) {
				drive_directions dir = coordList[i].x > coordList[i + 1].x ? rev : fwd;
				double angle = getTheta(coordList[i], coordList[i + 1], dir);
				double errorX = lookAhead * (sin(angle * M_PI / 180));
				double errorY = lookAhead * (cos(angle * M_PI / 180));
				Coordinate newDist = coordList[i];
				injectedList.push_back(coordList[i]);
				while(getDistance(coordList[i], newDist, fwd) < getDistance(coordList[i], coordList[i + 1], fwd)) {
					newDist.x += errorX * (dir ? -1 : 1);
					newDist.y += errorY * (dir ? -1 : 1);
					newDist.t = coordList[i + 1].t;
					newDist.facing = coordList[i + 1].facing;
					newDist.movement = MovementType::DRIVE;
					injectedList.push_back(newDist);
				}
				injectedList.pop_back();
			}
		}
		injectedList.push_back(coordList.back());
		return injectedList;
	}
	return coordList;
}

std::vector<Coordinate> smoothPath(std::vector<Coordinate> coordList, int lookAhead, int smoothing) {
	if(coordList.size() > 1) {
		int stops = coordList.size();
		for(int i = 0; i < smoothing; i++) {
			coordList.push_back(coordList.back());
		}
		std::vector<Coordinate> smoothList = coordList;
		Coordinate point_to_face;
		for(int i = 0; i < stops; i++) {
			bool checkSwing = false;
			for(int j = 1; j < smoothing + 1; j++) {
				if(coordList[i + j].movement == MovementType::SWING) {
					checkSwing = true;
					break;
				}
			}
			if(!checkSwing) {
				point_to_face = smoothList[i + smoothing];
				smoothList[i].t = getTheta(smoothList[i], point_to_face, smoothList[i + smoothing].facing);
			}
			drive_directions dir = coordList[i].x > point_to_face.x ? rev : fwd;
			double angle = getTheta(smoothList[i], point_to_face, dir);
			if(coordList[i + 1].movement != MovementType::SWING) {
				smoothList[i + 1].x = smoothList[i].x + (lookAhead * (sin(angle * M_PI / 180))) * (dir ? -1 : 1);
				smoothList[i + 1].y = smoothList[i].y + (lookAhead * (cos(angle * M_PI / 180))) * (dir ? -1 : 1);
			}
		}
		for(int i = 0; i < (smoothing); i++) {
			smoothList.pop_back();
		}
		return smoothList;
	}
	return coordList;
}

//
// Set position wrappers
//

void setPosition(double x, double y) {
	currentPoint.x = x;
	currentPoint.y = y;
	if(autonMode != AutonMode::BRAIN) chassis.odom_xy_set(currentPoint.x, currentPoint.y);
	currentPoint.movement = MovementType::TURN;
	autonPath.push_back(currentPoint);
}

void setPosition(double x, double y, double t) {
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = t;
	if(autonMode != AutonMode::BRAIN)  chassis.odom_xyt_set(currentPoint.x, currentPoint.y, t);
	currentPoint.movement = MovementType::TURN;
	autonPath.push_back(currentPoint);
}

//
// Wait wrappers
//

void pidWait(Wait type) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			switch(type) {
				case Wait::QUICK:
					chassis.pid_wait_quick();
					break;
				case Wait::CHAIN:
					chassis.pid_wait_quick_chain();
					break;
				default:
					chassis.pid_wait();
					break;
			}
			break;
		default:
			break;
	}
}

void pidWaitUntil(okapi::QLength distance) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_wait_until(distance);
			break;
		default:
			break;
	}
}

void pidWaitUntil(okapi::QAngle theta) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_wait_until(theta);
			break;
		default:
			break;
	}
}

void pidWaitUntil(Coordinate coordinate) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_wait_until({coordinate.x * okapi::inch, coordinate.y * okapi::inch});
			break;
		default:
			break;
	}
}

void delayMillis(int millis) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			pros::delay(millis);
			break;
		default:
			break;
	}
}

//
// Move to point wrappers
//

void moveToPoint(Coordinate newpoint, drive_directions direction, int speed) {
	bool slew_state = false;
	switch(autonMode) {
		case AutonMode::PLAIN:
			if(getDistance({chassis.odom_x_get(), chassis.odom_y_get()}, newpoint, direction) * okapi::inch > 24_in && speed > 90) slew_state = true;
			chassis.pid_turn_set((getTheta({chassis.odom_x_get(), chassis.odom_y_get()}, newpoint, direction)) * okapi::degree, speed);
			chassis.pid_wait_quick_chain();
			chassis.pid_drive_set(getDistance({chassis.odom_x_get(), chassis.odom_y_get()}, newpoint, direction) * okapi::inch, speed, slew_state);
			break;
		case AutonMode::ODOM:
			chassis.pid_odom_set({{newpoint.x * okapi::inch, newpoint.y * okapi::inch}, fwd, speed});
			break;
		default:
			break;
	}
	currentPoint.t = getTheta({currentPoint.x, currentPoint.y}, newpoint, direction);
	currentPoint.x = newpoint.x;
	currentPoint.y = newpoint.y;
	currentPoint.main = speed;
	currentPoint.facing = direction;
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

void moveToPoint(Coordinate currentpoint, Coordinate newpoint, drive_directions direction, int speed) {
	bool slew_state = false;
	switch(autonMode) {
		case AutonMode::PLAIN:
			if(getDistance({chassis.odom_x_get(), chassis.odom_y_get()}, newpoint, direction) * okapi::inch > 24_in && speed > 90) slew_state = true;
			chassis.pid_turn_set((getTheta({chassis.odom_x_get(), chassis.odom_y_get()}, newpoint, direction)) * okapi::degree, speed);
			chassis.pid_wait_quick_chain();
			chassis.pid_drive_set(getDistance({chassis.odom_x_get(), chassis.odom_y_get()}, newpoint, direction) * okapi::inch, speed, slew_state);
			break;
		case AutonMode::ODOM:
			chassis.pid_odom_set({{newpoint.x * okapi::inch, newpoint.y * okapi::inch}, fwd, speed});
			break;
		default:
			break;
	}
	currentPoint.x += newpoint.x - currentpoint.x;
	currentPoint.y += newpoint.y - currentpoint.y;
	currentPoint.t = getTheta({currentPoint.x, currentPoint.y}, newpoint, direction);
	currentPoint.main = speed;
	currentPoint.facing = direction;
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

//
// Drive set wrappers
//

void driveSet(double distance, int speed, bool slew) {
	double errorX = distance * (sin(chassis.odom_theta_get() * M_PI / 180));
	double errorY = distance * (cos(chassis.odom_theta_get() * M_PI / 180));
	drive_directions direction = distance < 0 ? rev : fwd;
	switch(autonMode) {
		case AutonMode::PLAIN:
			chassis.pid_drive_set(distance * okapi::inch, speed, false);
			break;
		case AutonMode::ODOM:
			chassis.pid_odom_set(distance * okapi::inch, speed, false);
			break;
		default:
			errorX = distance * (sin(currentPoint.t * M_PI / 180));
			errorY = distance * (cos(currentPoint.t * M_PI / 180));
			break;
	}
	currentPoint.x += errorX;
	currentPoint.y += errorY;
	currentPoint.main = speed;
	currentPoint.facing = direction;
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

void driveSet(double distance, int speed) {
	double errorX = distance * (sin(chassis.odom_theta_get() * M_PI / 180));
	double errorY = distance * (cos(chassis.odom_theta_get() * M_PI / 180));
	drive_directions direction = distance < 0 ? rev : fwd;
	switch(autonMode) {
		case AutonMode::PLAIN:
			chassis.pid_drive_set(distance * okapi::inch, speed, false);
			break;
		case AutonMode::ODOM:
			chassis.pid_odom_set(distance * okapi::inch, speed, false);
			break;
		default:
			errorX = distance * (sin(currentPoint.t * M_PI / 180));
			errorY = distance * (cos(currentPoint.t * M_PI / 180));
			break;
	}
	currentPoint.x += errorX;
	currentPoint.y += errorY;
	currentPoint.main = speed;
	currentPoint.facing = direction;
	currentPoint.movement = MovementType::DRIVE;
	autonPath.push_back(currentPoint);
}

//
// Turn set wrappers
//

void turnSet(double theta, int speed) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_turn_set(theta * okapi::degree, speed);
			break;
		default:
			break;
	}
	currentPoint.movement = MovementType::TURN;
	currentPoint.t = theta;
	autonPath.push_back(currentPoint);
}

void turnSet(double theta, int speed, e_angle_behavior behavior) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_turn_set(theta * okapi::degree, speed, behavior);
			break;
		default:
			break;
	}
	currentPoint.movement = MovementType::TURN;
	currentPoint.t = theta;
	autonPath.push_back(currentPoint);
}

void turnSet(Coordinate point, drive_directions direction, int speed) {
	double theta = getTheta(currentPoint, point, direction);
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_turn_set(theta * okapi::degree, speed);
			break;
		default:
			break;
	}
	currentPoint.movement = MovementType::TURN;
	currentPoint.t = theta;
	autonPath.push_back(currentPoint);
}

void turnSet(Coordinate point, drive_directions direction, int speed, e_angle_behavior behavior) {
	double theta = getTheta(currentPoint, point, direction);
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_turn_set(theta * okapi::degree, speed, behavior);
			break;
		default:
			break;
	}
	currentPoint.movement = MovementType::TURN;
	currentPoint.t = theta;
	autonPath.push_back(currentPoint);
}

void turnSetRelative(double theta, int speed) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			theta += chassis.odom_theta_get();
			chassis.pid_turn_set((theta)*okapi::degree, speed);
			break;
		default:
			theta += currentPoint.t;
			break;
	}
	currentPoint.movement = MovementType::TURN;
	currentPoint.t = theta;
	autonPath.push_back(currentPoint);
}

void turnSetRelative(double theta, int speed, e_angle_behavior behavior) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			theta += chassis.odom_theta_get();
			chassis.pid_turn_set((theta)*okapi::degree, speed, behavior);
			break;
		default:
			theta += currentPoint.t;
			break;
	}
	currentPoint.movement = MovementType::TURN;
	currentPoint.t = theta;
	autonPath.push_back(currentPoint);
}

//
// Swing set wrappers
//

void swingSet(e_swing side, double theta, double main, double opp, e_angle_behavior behavior) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_swing_set(side, theta * okapi::degree, main, opp, behavior);
			break;
		default:
			break;
	}
	double right = side == RIGHT_SWING ? main : opp;
	double left = side == LEFT_SWING ? main : opp;
	currentPoint = getArcFromTheta(currentPoint, side, behavior, right, left, theta);
	currentPoint.t = theta;
	currentPoint.movement = MovementType::SWING;
	currentPoint.side = side;
	currentPoint.main = main;
	currentPoint.opp = opp;
	currentPoint.behavior = behavior;
    currentPoint.facing = (side == LEFT_SWING && behavior == cw) || (side == RIGHT_SWING && behavior == ccw) ? fwd : rev;
	autonPath.push_back(currentPoint);
}

void swingSet(e_swing side, double theta, double main, e_angle_behavior behavior) {
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_swing_set(side, theta * okapi::degree, main, 0, behavior);
			break;
		default:
			break;
	}
	double right = side == RIGHT_SWING ? main : 0;
	double left = side == LEFT_SWING ? main : 0;
	currentPoint = getArcFromTheta(currentPoint, side, behavior, right, left, theta);
	currentPoint.t = theta;
	currentPoint.movement = MovementType::SWING;
	currentPoint.side = side;
	currentPoint.main = main;
	currentPoint.opp = 0;
	currentPoint.behavior = behavior;
    currentPoint.facing = (side == LEFT_SWING && behavior == cw) || (side == RIGHT_SWING && behavior == ccw) ? fwd : rev;
	autonPath.push_back(currentPoint);
}

void swingSet(e_swing side, double theta, double main, double opp) {
	e_angle_behavior behavior = ((theta > 180) && (side == RIGHT_SWING)) || ((theta < 180) && (side == LEFT_SWING)) ? cw : ccw;
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_swing_set(side, theta * okapi::degree, main, opp, behavior);
			break;
		default:
			break;
	}
	double right = side == RIGHT_SWING ? main : opp;
	double left = side == LEFT_SWING ? main : opp;
	currentPoint = getArcFromTheta(currentPoint, side, behavior, right, left, theta);
	currentPoint.t = theta;
	currentPoint.movement = MovementType::SWING;
	currentPoint.side = side;
	currentPoint.main = main;
	currentPoint.opp = opp;
	currentPoint.behavior = behavior;
    currentPoint.facing = (side == LEFT_SWING && behavior == cw) || (side == RIGHT_SWING && behavior == ccw) ? fwd : rev;
	autonPath.push_back(currentPoint);
}

void swingSet(e_swing side, double theta, double main) {
	e_angle_behavior behavior = ((theta > 180) && (side == RIGHT_SWING)) || ((theta < 180) && (side == LEFT_SWING)) ? cw : ccw;
	switch(autonMode) {
		case AutonMode::PLAIN:
		case AutonMode::ODOM:
			chassis.pid_swing_set(side, theta, main);
			break;
		default:
			break;
	}
	double right = side == RIGHT_SWING ? main : 0;
	double left = side == LEFT_SWING ? main : 0;
	currentPoint = getArcFromTheta(currentPoint, side, behavior, right, left, theta);
	currentPoint.t = theta;
	currentPoint.movement = MovementType::SWING;
	currentPoint.side = side;
	currentPoint.main = main;
	currentPoint.opp = 0;
	currentPoint.behavior = behavior;
    currentPoint.facing = (side == LEFT_SWING && behavior == cw) || (side == RIGHT_SWING && behavior == ccw) ? fwd : rev;
	autonPath.push_back(currentPoint);
}

//
// Print path
//

void getPath() {
	cout << "===========================================" << endl;
	for(auto point : autonPath) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}

void getPathInjected() {
	auto injected = injectPath(autonPath, 2);
	cout << "===========================================" << endl;
	for(auto point : injected) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}

void getPathSmooth() {
	auto smoothened = smoothPath(injectPath(autonPath, 1), 1, 4);
	cout << "===========================================" << endl;
	for(auto point : smoothened) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}