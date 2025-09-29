#include "main.h"  // IWYU pragma: keep

// // // // // // Tasks & Non-UI // // // // // //

//
// Object definitions
//

lv_obj_t* main_tv = lv_tileview_create(NULL);
lv_obj_t* autoSelector = lv_tileview_add_tile(main_tv, 0, 0, LV_DIR_NONE);
lv_obj_t* pidTuner = lv_tileview_add_tile(main_tv, 0, 1, LV_DIR_NONE);
lv_obj_t* pidTabview = lv_tabview_create(pidTuner, LV_DIR_TOP, 40);

vector<lv_obj_t*> screens{autoSelector, pidTuner};

lv_obj_t* colorInd = lv_obj_create(autoSelector);
lv_obj_t* colorOverlay = lv_img_create(autoSelector);
lv_obj_t* allianceInd = lv_obj_create(autoSelector);
lv_obj_t* allianceOverlay = lv_img_create(autoSelector);
lv_obj_t* pageUp = lv_label_create(autoSelector);
lv_obj_t* pageDown = lv_label_create(autoSelector);

lv_obj_t* autonTable = lv_list_create(autoSelector);
lv_obj_t* autonUp = lv_label_create(autoSelector);
lv_obj_t* autonDown = lv_label_create(autoSelector);
lv_obj_t* autonDesc = lv_label_create(autoSelector);
lv_obj_t* autonField = lv_img_create(autoSelector);
lv_obj_t* autonRobot = lv_img_create(autonField);
lv_obj_t* angleViewer;
lv_obj_t* angleText;

lv_obj_t* chassisTab = lv_tabview_add_tab(pidTabview, "chassis");
lv_obj_t* intakeTab = lv_tabview_add_tab(pidTabview, "intake");
lv_obj_t* driveTab = lv_tabview_add_tab(pidTabview, "drive");
lv_obj_t* turnTab = lv_tabview_add_tab(pidTabview, "turn");
lv_obj_t* swingTab = lv_tabview_add_tab(pidTabview, "swing");
lv_obj_t* headingTab = lv_tabview_add_tab(pidTabview, "heading");

lv_obj_t* motorPopup;

lv_style_t pushback;

LV_IMG_DECLARE(red_alliance);
LV_IMG_DECLARE(blue_alliance);
LV_IMG_DECLARE(colorindOverlay);
LV_IMG_DECLARE(robot);
LV_IMG_DECLARE(motorOverlay);

// // // // // // Tasks & Other Definitions // // // // // //

//
// Auton Selector
//

string controllerInput = "";
bool aligning = false;
bool playing = true;

AutonSel auton_sel;

void AutonSel::selector_populate(vector<AutonObj> auton_list) { autons.insert(autons.end(), auton_list.begin(), auton_list.end()); }

void angleCheckTask() {
	while(true) {
		if(aligning) {
			auto target = autonPath.size() > 0 ? autonPath[0].t : 0;
			auto current = fmod(chassis.odom_theta_get(), 360);
			lv_label_set_text(angleText,
							  (util::to_string_with_precision(current, 2) + " °" + "\ntarget: " + util::to_string_with_precision(target, 2)).c_str());
			if(target + 0.15 >= current && target - 0.15 <= current)
				lv_obj_set_style_bg_color(angleViewer, green, LV_PART_MAIN);
			else
				lv_obj_set_style_bg_color(angleViewer, red, LV_PART_MAIN);
		}
		pros::delay(10);
	}
}

int pathIter = 0;
vector<Coordinate> pathDisplay;

void resetViewer(bool full) {
	if(full) {
		auto preference = autonMode;
		autonMode = BRAIN;
		autonPath = {};
		auton_sel.selector_callback();
		pathDisplay = injectPath(autonPath, .5);
		getPathInjected();
		autonMode = preference;
		lv_img_set_src(autonField, &(allianceColor == BLUE ? blue_alliance : red_alliance));
	}
	pathIter = 0;
}

void pathViewerTask() {
	while(true) {
		if(pathIter < pathDisplay.size() && pathDisplay.size() > 1 && playing) {
			lv_obj_clear_flag(autonRobot, LV_OBJ_FLAG_HIDDEN);
			lv_obj_set_pos(autonRobot, (2 * pathDisplay[pathIter].x) - 11, 130 - (2 * pathDisplay[pathIter].y));
			if(pathIter < pathDisplay.size() - 1) {
				lv_img_set_angle(autonRobot, 10 * (pathDisplay[pathIter].t));
				if(pathDisplay[pathIter].left == KEY)
					pros::delay(pathDisplay[pathIter].right - 10);
				else {
					double velocity = (getVelocity(pathDisplay[pathIter].left) + getVelocity(pathDisplay[pathIter].right)) / 2;
					if(velocity == 0) velocity = getVelocity(pathDisplay[pathIter].left);
					pros::delay((1000 * abs(getTimeToPoint(.5, velocity))) - 10);
				}
			}
			if(pathIter == 1) pros::delay(490);
			pathIter++;
		} else if(pathIter >= pathDisplay.size()) {
			pros::delay(990);
			resetViewer(false);
		}
		pros::delay(10);
	}
}

//
// PID Tuner/Motor Info
//

vector<lv_obj_t*> motors_to_update{};

vector<MotorDisp> chassisMotors = {MotorDisp(&chassis.left_motors[0], "front left", lv_color_lighten(pink, 120), 50),
								   MotorDisp(&chassis.right_motors[0], "front right", lv_color_lighten(violet, 120), 50),
								   MotorDisp(&chassis.left_motors[1], "center left", lv_color_lighten(pink, 120), 50),
								   MotorDisp(&chassis.right_motors[1], "center right", lv_color_lighten(violet, 120), 50),
								   MotorDisp(&chassis.left_motors[2], "back left", lv_color_lighten(pink, 120), 50),
								   MotorDisp(&chassis.right_motors[2], "back right", lv_color_lighten(violet, 120), 50)};

vector<MotorDisp> intakeMotors = {MotorDisp(&intakeFirst, "first stage", lv_color_lighten(blue, 80), 50),
								  MotorDisp(&intakeSecond, "second stage", lv_color_lighten(blue, 80), 50),
								  MotorDisp(&intakeIndexer, "third stage", lv_color_lighten(blue, 80), 50)};

MotorTab chassisTabObj = 
	MotorTab("chassis", theme_color, &chassis.leftPID.error, 24, chassisMotors, drive_test, false,
								  PidTunerValues(0.25, 0.05, 0.25, &chassis.fwd_rev_drivePID), chassisTab);
MotorTab intakeTabObj = 
	MotorTab("intake", theme_color, &chassis.leftPID.error, 24, intakeMotors, drive_test, false,
								 PidTunerValues(0.25, 0.05, 0.25, &chassis.fwd_rev_drivePID), intakeTab);
MotorTab driveTabObj = 
	MotorTab("drive PID", theme_color, &chassis.leftPID.error, 24, chassisMotors, drive_test, true,
								PidTunerValues(0.25, 0.05, 0.25, &chassis.fwd_rev_drivePID), driveTab);
MotorTab turnTabObj =
	MotorTab("turn PID", theme_color, &chassis.turnPID.error, 360, chassisMotors, turn_test, true, PidTunerValues(0.25, 0.05, 0.25, &chassis.turnPID), turnTab);
MotorTab swingTabObj = 
	MotorTab("swing PID", theme_color, &chassis.swingPID.error, 90, chassisMotors, swing_test, true,
								PidTunerValues(0.25, 0.05, 0.25, &chassis.fwd_rev_swingPID), swingTab);
MotorTab headingTabObj = 
	MotorTab("heading PID", theme_color, &chassis.turnPID.error, 180, chassisMotors, heading_test, true,
								  PidTunerValues(0.25, 0.05, 0.25, &chassis.headingPID), headingTab);

MotorTab* selectedTabObj = &driveTabObj;

vector<MotorTab> tabList{chassisTabObj, intakeTabObj, driveTabObj, turnTabObj, swingTabObj, headingTabObj};

bool reading = false;
bool probing = false;

MotorTab* current_tab = &driveTabObj;

vector<lv_coord_t> errorData;

void motorUpdateTask() {
	int cnt = 0;
	ofstream swingVelo;
	if(pros::usd::is_installed()) swingVelo.open("/usd/swing_curve.txt", ios::out | ios::app);
	if(swingVelo.is_open()) {
		swingVelo << "=====================================\n";
	}
	while(true) {
		if(lv_tileview_get_tile_act(main_tv) == pidTuner) {
			for(auto motor_to_update : motors_to_update) {
				lv_event_send(motor_to_update, LV_EVENT_REFRESH, nullptr);
			}
		}

		if(reading) {
			lv_event_send(motorPopup, LV_EVENT_REFRESH, nullptr);
		}

		if(probing) {
			errorData.push_back(*current_tab->error);

			if(pros::usd::is_installed() && current_tab->name == "swing PID") {
				if(swingVelo.is_open()) {
					swingVelo << "(" << cnt << ", " << chassis.drive_get()[0] << "), (" << cnt << ", " << chassis.drive_get()[1] << ")\n";
				}
			}
			cnt++;
		} else
			cnt = 0;
		pros::delay(10);
	}
}

// // // // // // UI // // // // // //

//
// Main UI
//

int scrpage = 0;

static void pageUpEvent(lv_event_t* e) {
	scrpage++;
	scrpage %= screens.size();
	lv_obj_set_tile(main_tv, screens[scrpage], LV_ANIM_ON);
	lv_obj_set_parent(pageUp, screens[scrpage]);
	lv_obj_set_parent(pageDown, screens[scrpage]);
}

static void pageDownEvent(lv_event_t* e) {
	scrpage--;
	if(scrpage < 0) scrpage += screens.size();
	lv_obj_set_tile(main_tv, screens[scrpage], LV_ANIM_ON);
	lv_obj_set_parent(pageUp, screens[scrpage]);
	lv_obj_set_parent(pageDown, screens[scrpage]);
}

lv_event_cb_t PageUpEvent = pageUpEvent;
lv_event_cb_t PageDownEvent = pageDownEvent;

void uiInit() {
	// Initialize style
	lv_style_init(&pushback);
	lv_style_set_bg_color(&pushback, theme_color);
	lv_style_set_outline_color(&pushback, theme_accent);
	lv_style_set_text_color(&pushback, theme_accent);
	lv_style_set_bg_opa(&pushback, 255);
	lv_style_set_outline_width(&pushback, 3);
	lv_style_set_border_width(&pushback, 0);
	lv_style_set_text_font(&pushback, &lv_font_montserrat_16);
	lv_style_set_radius(&pushback, 0);

	// Set up flags
	lv_obj_clear_flag(colorInd, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_clear_flag(allianceInd, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(pageUp, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(pageDown, LV_OBJ_FLAG_CLICKABLE);

	// Add styles
	lv_obj_add_style(main_tv, &pushback, LV_PART_MAIN);
	lv_obj_add_style(autoSelector, &pushback, LV_PART_MAIN);
	lv_obj_add_style(pidTuner, &pushback, LV_PART_MAIN);
	lv_obj_add_style(colorInd, &pushback, LV_PART_MAIN);
	lv_obj_add_style(allianceInd, &pushback, LV_PART_MAIN);
	lv_obj_set_scrollbar_mode(main_tv, LV_SCROLLBAR_MODE_OFF);
	lv_obj_set_style_bg_opa(autoSelector, 255, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(pidTuner, 255, LV_PART_MAIN);
	lv_obj_set_style_outline_opa(autoSelector, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_opa(pidTuner, 0, LV_PART_MAIN);

	// Initialize screens
	autoSelectorInit();
	pidTunerInit();

	// Set up ring indicator
	lv_img_set_src(colorOverlay, &colorindOverlay);
	lv_img_set_src(allianceOverlay, &colorindOverlay);
	lv_obj_set_size(colorInd, 51, 51);
	lv_obj_set_size(allianceInd, 51, 51);
	lv_obj_set_pos(colorInd, 421, 20);
	lv_obj_set_pos(colorOverlay, 421, 20);
	lv_obj_set_pos(allianceInd, 368, 20);
	lv_obj_set_pos(allianceOverlay, 368, 20);
	lv_obj_set_style_outline_width(colorInd, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(colorOverlay, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(allianceInd, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(allianceOverlay, 0, LV_PART_MAIN);
	lv_obj_set_style_img_recolor_opa(colorOverlay, 255, LV_PART_MAIN);
	lv_obj_set_style_img_recolor_opa(allianceOverlay, 255, LV_PART_MAIN);
	lv_obj_move_foreground(colorInd);
	lv_obj_move_foreground(allianceInd);
	lv_obj_move_foreground(colorOverlay);
	lv_obj_move_foreground(allianceOverlay);
	colorSet(allianceColor, allianceInd);

	// Set up page up/down
	lv_label_set_text(pageUp, LV_SYMBOL_UP);
	lv_label_set_text(pageDown, LV_SYMBOL_DOWN);
	lv_obj_set_style_text_opa(pageUp, 128, LV_STATE_PRESSED);
	lv_obj_set_style_text_opa(pageDown, 128, LV_STATE_PRESSED);
	lv_obj_set_style_text_font(pageUp, &lv_font_montserrat_16, LV_PART_MAIN);
	lv_obj_set_style_text_font(pageDown, &lv_font_montserrat_16, LV_PART_MAIN);
	lv_obj_set_style_outline_width(pageUp, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(pageDown, 0, LV_PART_MAIN);
	lv_obj_align(pageUp, LV_ALIGN_TOP_MID, 0, 0);
	lv_obj_align(pageDown, LV_ALIGN_BOTTOM_MID, 0, 0);
	lv_obj_add_event_cb(pageUp, PageUpEvent, LV_EVENT_CLICKED, nullptr);
	lv_obj_add_event_cb(pageDown, PageDownEvent, LV_EVENT_CLICKED, nullptr);

	// Load main screen
	lv_scr_load(main_tv);
}

//
// Auton Selector UI
//

static void selectAuton(lv_event_t* e) {
	AutonObj* getAuton = (AutonObj*)lv_event_get_user_data(e);
	lv_obj_t* target = lv_event_get_target(e);
	for(int i = 0; i < lv_obj_get_child_cnt(autonTable); i++) {
		lv_obj_t* auton = lv_obj_get_child(autonTable, i);
		lv_obj_clear_state(auton, LV_STATE_CHECKED);
	}
	lv_obj_add_state(target, LV_STATE_CHECKED);
	lv_label_set_text(autonDesc, ((*getAuton).desc).c_str());
	lv_obj_set_style_bg_color(autonDesc, lv_color_darken((*getAuton).color, 80), LV_PART_MAIN);
	lv_obj_set_style_img_recolor(colorOverlay, lv_color_darken((*getAuton).color, 80), LV_PART_MAIN);
	lv_obj_set_style_img_recolor(allianceOverlay, lv_color_darken((*getAuton).color, 80), LV_PART_MAIN);
	auton_sel.selector_callback = (*getAuton).callback;
	auton_sel.selector_name = (*getAuton).name;
	resetViewer(true);
}

static void autonUpEvent(lv_event_t* e) { lv_obj_scroll_by_bounded(autonTable, 0, lv_obj_get_height(autonTable), LV_ANIM_ON); }

static void autonDownEvent(lv_event_t* e) { lv_obj_scroll_by_bounded(autonTable, 0, -lv_obj_get_height(autonTable), LV_ANIM_ON); }

static void angleCheckCloseEvent(lv_event_t* e) { aligning = false; }

lv_event_cb_t AngleCheckCloseEvent = angleCheckCloseEvent;

static void angleCheckEvent(lv_event_t* e) {
	angleViewer = lv_msgbox_create(NULL, "check alignment", "0°", NULL, true);
	angleText = lv_msgbox_get_text(angleViewer);
	aligning = true;

	lv_obj_add_event_cb(lv_msgbox_get_close_btn(angleViewer), AngleCheckCloseEvent, LV_EVENT_PRESSED, NULL);
	lv_obj_add_style(lv_msgbox_get_close_btn(angleViewer), &pushback, LV_PART_MAIN);
	lv_obj_add_style(angleViewer, &pushback, LV_PART_MAIN);
	lv_obj_set_style_text_font(angleViewer, &lv_font_montserrat_30, LV_PART_MAIN);
	lv_obj_set_style_text_font(lv_msgbox_get_title(angleViewer), &lv_font_montserrat_14, LV_PART_MAIN);
	lv_obj_set_style_text_font(lv_msgbox_get_close_btn(angleViewer), &lv_font_montserrat_24, LV_PART_MAIN);
	lv_obj_set_width(angleViewer, 300);
	lv_obj_align(angleViewer, LV_ALIGN_CENTER, 0, 0);
}

static void pauseEvent(lv_event_t* e) {
	auto event = lv_event_get_code(e);
	if(event == LV_EVENT_PRESSING) playing = false;
	if(event == LV_EVENT_CLICKED) playing = true;
}

static void colorEvent(lv_event_t* e) {
	setAlliance((Colors)(((int)allianceColor + 1) % 3));
	colorSet(allianceColor, allianceInd);
	resetViewer(true);
}

lv_event_cb_t SelectAuton = selectAuton;
lv_event_cb_t AutonUpEvent = autonUpEvent;
lv_event_cb_t AutonDownEvent = autonDownEvent;
lv_event_cb_t AngleCheckEvent = angleCheckEvent;
lv_event_cb_t PauseEvent = pauseEvent;
lv_event_cb_t ColorEvent = colorEvent;

void autoSelectorInit() {
	// Add base styles
	lv_obj_add_style(autonTable, &pushback, LV_PART_MAIN);
	lv_obj_add_style(autonTable, &pushback, LV_PART_ITEMS);
	lv_obj_add_style(autonDesc, &pushback, LV_PART_MAIN);
	lv_obj_add_style(autonField, &pushback, LV_PART_MAIN);
	lv_obj_add_style(autonRobot, &pushback, LV_PART_MAIN);
	lv_obj_add_style(autonUp, &pushback, LV_PART_MAIN);
	lv_obj_add_style(autonDown, &pushback, LV_PART_MAIN);

	// Set image sources and default text
	lv_img_set_src(autonField, &red_alliance);
	lv_img_set_src(autonRobot, &robot);
	lv_label_set_text(autonDesc, "No auton selected");
	lv_label_set_text(autonUp, LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP);
	lv_label_set_text(autonDown, LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN);

	// Set flags
	lv_obj_add_flag(autonUp, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(autonDown, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(autonField, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(allianceInd, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(allianceOverlay, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(autonRobot, LV_OBJ_FLAG_HIDDEN);

	// Set sizes of objects
	lv_obj_set_size(autonTable, 160, 206);
	lv_obj_set_size(autonDesc, 288, 57);
	lv_obj_set_size(autonField, 288, 144);

	// Align and set positions of objects
	lv_obj_align(autonTable, LV_ALIGN_LEFT_MID, 5, 0);
	lv_obj_align(autonDesc, LV_ALIGN_TOP_RIGHT, -5, 17);
	lv_obj_align(autonField, LV_ALIGN_BOTTOM_RIGHT, -5, -17);
	lv_obj_align(autonUp, LV_ALIGN_CENTER, -64, -68);
	lv_obj_align(autonDown, LV_ALIGN_CENTER, -64, 68);

	// Modify styles

	lv_obj_set_style_text_opa(autonUp, 128, LV_STATE_PRESSED);
	lv_obj_set_style_text_opa(autonDown, 128, LV_STATE_PRESSED);
	lv_obj_set_style_bg_opa(autonRobot, 0, LV_PART_MAIN);

	lv_obj_set_style_outline_width(autonTable, 1, LV_PART_ITEMS);
	lv_obj_set_style_outline_width(autonField, 5, LV_STATE_PRESSED);
	lv_obj_set_style_outline_width(autonRobot, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(autonUp, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(autonDown, 0, LV_PART_MAIN);

	lv_obj_set_style_text_font(autonUp, &lv_font_montserrat_16, LV_PART_MAIN);
	lv_obj_set_style_text_font(autonDown, &lv_font_montserrat_16, LV_PART_MAIN);
	lv_obj_set_style_text_line_space(autonUp, -12, LV_PART_MAIN);
	lv_obj_set_style_text_line_space(autonDown, -12, LV_PART_MAIN);
	lv_obj_set_style_pad_all(autonDesc, 5, LV_PART_MAIN);
	lv_obj_set_style_pad_right(autonDesc, 111, LV_PART_MAIN);
	lv_obj_set_style_pad_hor(autonTable, 0, LV_PART_MAIN);

	lv_obj_set_scrollbar_mode(autonTable, LV_SCROLLBAR_MODE_OFF);

	// Add events
	lv_obj_add_event_cb(autonUp, AutonUpEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(autonDown, AutonDownEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(autonField, AngleCheckEvent, LV_EVENT_SHORT_CLICKED, NULL);
	lv_obj_add_event_cb(autonField, PauseEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(autonField, PauseEvent, LV_EVENT_PRESSING, NULL);
	lv_obj_add_event_cb(allianceInd, colorEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(allianceOverlay, colorEvent, LV_EVENT_CLICKED, NULL);

	// Set up list
	for(int i = 0; i < auton_sel.autons.size(); i++) {
		lv_obj_t* new_auto = lv_list_add_btn(autonTable, NULL, (auton_sel.autons[i].name).c_str());
		lv_obj_add_style(new_auto, &pushback, LV_PART_MAIN);
		lv_obj_set_style_text_font(new_auto, &pros_font_dejavu_mono_18, LV_PART_MAIN);
		lv_obj_set_style_bg_color(new_auto, lv_color_darken(auton_sel.autons[i].color, 80), LV_PART_MAIN);
		lv_obj_set_style_outline_width(new_auto, 1, LV_PART_MAIN);
		lv_obj_set_style_outline_width(new_auto, 4, LV_STATE_CHECKED);
		lv_obj_set_style_outline_width(new_auto, 6, LV_STATE_PRESSED);
		lv_obj_set_style_bg_opa(new_auto, 220, LV_STATE_CHECKED);
		lv_obj_set_style_bg_opa(new_auto, 180, LV_STATE_PRESSED);
		lv_obj_set_style_pad_hor(new_auto, 8, LV_PART_MAIN);
		lv_obj_add_event_cb(new_auto, SelectAuton, LV_EVENT_CLICKED, &auton_sel.autons[i]);
	}
}

//
// PID Tuner/Motor Info
//

static void selectTab(lv_event_t* e) {
	auto tabindex = lv_tabview_get_tab_act(pidTabview);
	selectedTabObj = (MotorTab*)lv_obj_get_user_data(tabList[tabindex].tab);
	cout << "selected " << tabindex << "\n";
}

static void motorPopupCloseEvent(lv_event_t* e) { reading = false; }

lv_event_cb_t MotorPopupCloseEvent = motorPopupCloseEvent;

static void motorUpdateEvent(lv_event_t* e) {
	MotorDisp* current = (MotorDisp*)lv_event_get_user_data(e);
	lv_obj_t* object = lv_event_get_target(e);

	double temperature = current->motor->get_temperature();
	double ratio = pow(1.05, (temperature + 60)) - (0.5 * temperature) - 10;
	if(temperature > 255 || ratio > 255) ratio = 255;

	lv_color32_t mixed_color = lv_color_mix(current->color, current->bg_color, ratio);
	if(temperature > current->maxtemp) {
		if(current->motor->get_temperature() > 255)
			mixed_color = lv_color_make(-current->color.ch.red, -current->color.ch.green, -current->color.ch.blue);
		else
			mixed_color = red;
	}

	lv_obj_set_style_img_recolor(object, lv_color_darken(mixed_color, 60), LV_STATE_PRESSED);
	lv_obj_set_style_img_recolor(object, mixed_color, LV_PART_MAIN);
}

static void motorPopupUpdate(lv_event_t* e) {
	MotorDisp* current = (MotorDisp*)lv_event_get_user_data(e);

	double temperature = current->motor->get_temperature();
	double ratio = pow(1.05, (temperature + 60)) - (0.5 * temperature) - 10;
	if(temperature > 255 || ratio > 255) ratio = 255;

	lv_color32_t mixed_color = lv_color_mix(current->color, current->bg_color, ratio);
	if(temperature > current->maxtemp) {
		if(current->motor->get_temperature() > 255)
			mixed_color = lv_color_make(-current->color.ch.red, -current->color.ch.green, -current->color.ch.blue);
		else
			mixed_color = red;
	}

	lv_label_set_text(lv_msgbox_get_text(motorPopup),
					  current->motor->get_temperature() < 255 ? (std::to_string((int)(current->motor->get_temperature())) + "°C").c_str() : "Unplugged");
	lv_obj_set_style_text_font(motorPopup, current->motor->get_temperature() < 255 ? &lv_font_montserrat_36 : &lv_font_montserrat_30, LV_PART_MAIN);
	lv_obj_set_style_bg_color(motorPopup, mixed_color, LV_PART_MAIN);
}

static void motorUpEvent(lv_event_t* e) {
	lv_obj_t* cont = (lv_obj_t*)lv_event_get_user_data(e);
	lv_obj_scroll_by_bounded(cont, 0, 166, LV_ANIM_ON);
}

static void motorDownEvent(lv_event_t* e) {
	lv_obj_t* cont = (lv_obj_t*)lv_event_get_user_data(e);
	lv_obj_scroll_by_bounded(cont, 0, -166, LV_ANIM_ON);
}

static void motorMsgboxEvent(lv_event_t* e) {
	MotorDisp* current = (MotorDisp*)lv_event_get_user_data(e);

	motorPopup = lv_msgbox_create(
		NULL, current->name.c_str(),
		current->motor->get_temperature() < 255 ? (std::to_string((int)(current->motor->get_temperature())) + "°C").c_str() : "Unplugged", nullptr, true);
	lv_obj_t* popupClose = lv_msgbox_get_close_btn(motorPopup);

	lv_obj_add_style(popupClose, &pushback, LV_PART_MAIN);
	lv_obj_add_style(motorPopup, &pushback, LV_PART_MAIN);

	lv_obj_set_style_text_font(lv_msgbox_get_title(motorPopup), &lv_font_montserrat_14, LV_PART_MAIN);
	lv_obj_set_style_text_font(motorPopup, current->motor->is_installed() ? &lv_font_montserrat_36 : &lv_font_montserrat_30, LV_PART_MAIN);
	lv_obj_set_style_text_color(motorPopup, theme_accent, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(motorPopup, 255, LV_PART_MAIN);
	lv_obj_align(motorPopup, LV_ALIGN_CENTER, 0, 0);

	lv_obj_set_style_text_font(popupClose, &lv_font_montserrat_24, LV_PART_MAIN);
	lv_obj_set_style_text_color(popupClose, theme_accent, LV_PART_MAIN);
	lv_obj_set_style_border_opa(popupClose, 0, LV_PART_MAIN);

	lv_obj_add_event_cb(popupClose, MotorPopupCloseEvent, LV_EVENT_PRESSED, nullptr);
	lv_obj_add_event_cb(motorPopup, motorPopupUpdate, LV_EVENT_REFRESH, current);
	reading = true;
}

static void pidProbeEvent(lv_event_t* e) {
	current_tab = (MotorTab*)lv_event_get_user_data(e);
	lv_obj_t* graph = lv_event_get_target(e);

	if(!pros::competition::is_connected()) {
		errorData = {};
		probing = true;
		chassis.pid_targets_reset();
		chassis.drive_imu_reset();
		chassis.drive_sensor_reset();
		pros::motor_brake_mode_e_t brakePreference = chassis.drive_brake_get();
		double activebrakePreference = chassis.opcontrol_drive_activebrake_get();
		chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
		chassis.opcontrol_drive_activebrake_set(0.0);
		current_tab->callback(current_tab->target);
		chassis.pid_wait();
		probing = false;
		chassis.drive_brake_set(brakePreference);
		chassis.opcontrol_drive_activebrake_set(activebrakePreference);
		chassis.opcontrol_drive_sensors_reset();

		int start = errorData.size() - 201;
		if(start < 0) start = 0;

		for(int i = 0; i < 200; i++) {
			auto datapoint = start + i < errorData.size() ? errorData[start + i] : errorData.back();
			current_tab->errorPoints[i] = -datapoint;
		}

		lv_chart_refresh(graph);
	}
}

lv_event_cb_t SelectTab = selectTab;
lv_event_cb_t MotorUpdateEvent = motorUpdateEvent;
lv_event_cb_t MotorMsgboxEvent = motorMsgboxEvent;
lv_event_cb_t MotorUpEvent = motorUpEvent;
lv_event_cb_t MotorDownEvent = motorDownEvent;
lv_event_cb_t PidProbeEvent = pidProbeEvent;

void MotorTab::addTab() {
	// Create main container for screen
	lv_obj_add_style(this->tab, &pushback, LV_PART_MAIN);
	lv_obj_set_style_bg_color(this->tab, color, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(this->tab, 255, LV_PART_MAIN);
	lv_obj_set_style_outline_opa(this->tab, 0, LV_PART_MAIN);
	lv_obj_clear_flag(this->tab, LV_OBJ_FLAG_SCROLLABLE);

	// Create motor sub-container
	lv_obj_t* motorbox = lv_obj_create(this->tab);
	lv_obj_t* motorTabUp = lv_label_create(this->tab);
	lv_obj_t* motorTabDown = lv_label_create(this->tab);
	lv_obj_add_style(motorbox, &pushback, LV_PART_MAIN);
	lv_obj_add_style(motorTabUp, &pushback, LV_PART_MAIN);
	lv_obj_add_style(motorTabDown, &pushback, LV_PART_MAIN);
	lv_obj_set_size(motorbox, this->usePid ? 122 : 470, 168);
	lv_obj_align(motorbox, LV_ALIGN_TOP_LEFT, -17, -17);
	lv_obj_align_to(motorTabUp, motorbox, LV_ALIGN_RIGHT_MID, 33, -57);
	lv_obj_align_to(motorTabDown, motorbox, LV_ALIGN_RIGHT_MID, 33, 33);

	lv_label_set_text(motorTabUp, LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP);
	lv_label_set_text(motorTabDown, LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN);
	lv_obj_set_style_text_font(motorTabUp, &lv_font_montserrat_16, LV_PART_MAIN);
	lv_obj_set_style_text_font(motorTabDown, &lv_font_montserrat_16, LV_PART_MAIN);
	lv_obj_set_style_text_line_space(motorTabUp, -12, LV_PART_MAIN);
	lv_obj_set_style_text_line_space(motorTabDown, -12, LV_PART_MAIN);
	lv_obj_set_style_outline_width(motorTabUp, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(motorTabDown, 0, LV_PART_MAIN);
	lv_obj_add_flag(motorTabUp, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(motorTabDown, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_set_style_text_opa(motorTabUp, 128, LV_STATE_PRESSED);
	lv_obj_set_style_text_opa(motorTabDown, 128, LV_STATE_PRESSED);
	lv_obj_add_event_cb(motorTabUp, MotorUpEvent, LV_EVENT_CLICKED, motorbox);
	lv_obj_add_event_cb(motorTabDown, MotorDownEvent, LV_EVENT_CLICKED, motorbox);

	lv_obj_set_layout(motorbox, LV_LAYOUT_FLEX);
	lv_obj_set_flex_align(motorbox, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
	lv_obj_set_flex_flow(motorbox, LV_FLEX_FLOW_ROW_WRAP);
	lv_obj_set_style_pad_all(motorbox, 3, LV_PART_MAIN);
	lv_obj_set_style_pad_column(motorbox, 16, LV_PART_MAIN);
	lv_obj_set_style_pad_row(motorbox, 2, LV_PART_MAIN);
	lv_obj_set_scrollbar_mode(motorbox, LV_SCROLLBAR_MODE_ON);
	lv_obj_clear_flag(motorbox, LV_OBJ_FLAG_SCROLLABLE);

	for(int i = 0; i < this->motors.size(); i++) {
		this->motors[i].bg_color = this->color;

		lv_obj_t* motorIcon = lv_img_create(motorbox);

		lv_obj_add_style(motorIcon, &pushback, LV_PART_MAIN);
		lv_obj_set_style_outline_width(motorIcon, 0, LV_PART_MAIN);
		lv_obj_set_size(motorIcon, 100, 80);
		lv_img_set_src(motorIcon, &motorOverlay);

		lv_obj_set_style_img_recolor_opa(motorIcon, 255, LV_PART_MAIN);

		lv_obj_add_flag(motorIcon, LV_OBJ_FLAG_CLICKABLE);
		lv_obj_clear_flag(motorIcon, LV_OBJ_FLAG_SCROLLABLE);

		lv_obj_add_event_cb(motorIcon, MotorUpdateEvent, LV_EVENT_REFRESH, &this->motors[i]);
		lv_obj_add_event_cb(motorIcon, MotorMsgboxEvent, LV_EVENT_CLICKED, &this->motors[i]);
		motors_to_update.push_back(motorIcon);
	}

	if(this->usePid == false) return;

	// Create and initialize PID tuner graph
	lv_obj_t* pidGraph = lv_chart_create(this->tab);
	lv_obj_add_style(pidGraph, &pushback, LV_PART_MAIN);
	this->graph = pidGraph;

	lv_obj_set_size(pidGraph, 347, 166);
	lv_obj_align_to(pidGraph, motorbox, LV_ALIGN_OUT_RIGHT_MID, 0, 0);

	lv_chart_set_range(pidGraph, LV_CHART_AXIS_PRIMARY_Y, -this->target, this->target);
	lv_obj_set_style_size(pidGraph, 0, LV_PART_INDICATOR);
	lv_chart_set_point_count(pidGraph, 200);

	lv_chart_series_t* series = lv_chart_add_series(pidGraph, violet, LV_CHART_AXIS_PRIMARY_Y);

	lv_chart_set_ext_y_array(pidGraph, series, (lv_coord_t*)this->errorPoints);

	lv_obj_add_flag(pidGraph, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_event_cb(pidGraph, PidProbeEvent, LV_EVENT_CLICKED, this);
}

void pidTunerInit() {
	lv_obj_t* tabs = lv_tabview_get_tab_btns(pidTabview);
	lv_obj_t* container = lv_tabview_get_content(pidTabview);

	// Add base styles
	lv_obj_add_style(pidTabview, &pushback, LV_PART_MAIN);
	lv_obj_add_style(tabs, &pushback, LV_PART_ITEMS);
	lv_obj_add_style(tabs, &pushback, LV_PART_ITEMS | LV_STATE_CHECKED);
	lv_obj_add_style(container, &pushback, LV_PART_MAIN);

	// Set flags
	lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

	// Add tabs
	chassisTabObj.addTab();
	intakeTabObj.addTab();
	driveTabObj.addTab();
	turnTabObj.addTab();
	swingTabObj.addTab();
	headingTabObj.addTab();

	// Modify styles
	lv_obj_set_style_text_font(tabs, &pros_font_dejavu_mono_18, LV_PART_ITEMS);
	lv_obj_set_style_text_font(tabs, &pros_font_dejavu_mono_18, LV_PART_ITEMS | LV_STATE_CHECKED);
	lv_obj_set_style_bg_color(tabs, theme_color, LV_PART_MAIN);
	lv_obj_set_style_bg_color(tabs, lv_color_lighten(theme_color, 60), LV_PART_ITEMS | LV_STATE_CHECKED);
	lv_obj_set_style_bg_opa(tabs, 255, LV_PART_MAIN);
	lv_obj_set_style_border_color(tabs, theme_accent, LV_PART_ITEMS | LV_STATE_CHECKED);
	lv_obj_set_style_border_opa(tabs, 255, LV_PART_ITEMS | LV_STATE_CHECKED);
	lv_obj_set_style_border_side(tabs, LV_BORDER_SIDE_INTERNAL, LV_PART_ITEMS | LV_STATE_CHECKED);
	lv_obj_set_style_border_width(tabs, 5, LV_PART_ITEMS | LV_STATE_CHECKED);
	lv_obj_set_style_outline_width(tabs, 3, LV_PART_ITEMS);
	lv_obj_set_style_outline_width(container, 3, LV_PART_MAIN);
	lv_obj_set_style_outline_width(pidTabview, 3, LV_PART_MAIN);

	// Align and set positions of objects
	lv_obj_set_size(pidTabview, 470, 206);
	lv_obj_align(pidTabview, LV_ALIGN_CENTER, 0, 0);

	// Add user data
	lv_obj_set_user_data(chassisTab, &chassisTabObj);
	lv_obj_set_user_data(intakeTab, &intakeTabObj);
	lv_obj_set_user_data(driveTab, &driveTabObj);
	lv_obj_set_user_data(turnTab, &turnTabObj);
	lv_obj_set_user_data(swingTab, &swingTabObj);
	lv_obj_set_user_data(headingTab, &headingTabObj);

	lv_obj_add_event_cb(pidTabview, SelectTab, LV_EVENT_VALUE_CHANGED, nullptr);
}