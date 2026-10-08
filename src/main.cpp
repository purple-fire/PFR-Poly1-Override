#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.h"
using namespace pros;

Controller controller(E_CONTROLLER_MASTER);
MotorGroup right_mg({-1, 2, -3, 4}, MotorGearset::blue);
MotorGroup left_mg({5, -6, 7, -8}, MotorGearset::blue);
MotorGroup intake_mg({9, -10}, MotorGearset::blue);
MotorGroup slides_mg({11, 20}, MotorGearset::blue);
//slides_mg.set_brake_modes(E_MOTOR_BRAKE_HOLD);


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(10);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "slide: %d", controller.get_digital(E_CONTROLLER_DIGITAL_L1) - controller.get_digital(E_CONTROLLER_DIGITAL_L2));
            pros::lcd::print(4, "intake: %d", controller.get_digital(E_CONTROLLER_DIGITAL_R1) - controller.get_digital(E_CONTROLLER_DIGITAL_R2));
            // delay to save resources
            pros::delay(20);
        }
    });
}

void on_center_button() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {


}

void opcontrol() {
	while (true) {
		// Arcade control scheme
		float slow = controller.get_digital(E_CONTROLLER_DIGITAL_A) ? 1.0 : 0.5;
		int slide = (controller.get_digital(E_CONTROLLER_DIGITAL_L1) - controller.get_digital(E_CONTROLLER_DIGITAL_L2)) * 127;
		int intake = (controller.get_digital(E_CONTROLLER_DIGITAL_R1) - controller.get_digital(E_CONTROLLER_DIGITAL_R2)) * 127;
		int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = -controller.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
		chassis.arcade(dir, turn);
		left_mg.move((dir - turn) * slow);                      // Sets left motor voltage
		right_mg.move((dir + turn) * slow);                     // Sets right motor voltage
        slides_mg.move(slide);
        if (!slide) {
            slides_mg.brake();
        }
        intake_mg.move(intake);
        if (!intake) {
            intake_mg.brake();
        }
		delay(20);							// Run for 20 ms then update
	}
}