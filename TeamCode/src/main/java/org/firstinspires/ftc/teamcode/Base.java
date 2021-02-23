/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class Base extends LinearOpMode {
	DcMotor lf, rf, lb, rb;
	BNO055IMU imu;

	run opmode
		init motors
		runmotors()

	runmotors(inputs)
		motor.power(input)

	void initializeHardware() {
		lf = hardwareMap.dcMotor.get("leftFrontMotor");
		lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rf = hardwareMap.dcMotor.get("rightFrontMotor");
		rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lb = hardwareMap.dcMotor.get("leftBackMotor");
		lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rb = hardwareMap.dcMotor.get("rightBackMotor");
		rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		BNO055IMU.Parameters params = new BNO055IMU.Parameters();

		params.mode = BNO055IMU.SensorMode.IMU;
		params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		params.accelUnit = BNO055IMU.AccelUnit/METERS_PERSEC_PERSEC;
		params.loggingEnabled = false;

		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(params);
	}

	void runMotors(double lfpower, double rfpower, double lbpower, double rbpower) {
		lf.setPower(lfpower);
		rf.setPower(rfpower);
		lb.setPower(lbpower);
		rb.setPower(rbpower);
	}

	double getEncoderPosition(byte motor) {
	switch(motor) {
		case 0: return lf.getEncoderPosition();
				break;
		case 1: return rf.getEncoderPosition();
				break;
		case 2: return lb.getEncoderPosition();
				break;
		case 3: return rb.getEncoderPosition();
				break;
			
	
}

class BaseHelper {
	double xinput;
	double yinput;
	double rotinput;

	getpowervalues(PID or joystick inputs)
		return math shit
	getcorrection(imu input)
		return more math shit
	getoutput(powervalues, correction)
		return final motor powers
	
	void getControllerInput(xin, yin, rotin) {
		xinput = xin;
		yinput = yin;
		rotinput = rotin;
	}
}

class Auton {
	bot = Base
	bothelper = BaseHelper
	run opmode
		bot.init
		waitforstart
		loop
			bot.runmotors(bothelper.getoutput(bothelper.getpowervalues(PID output), bothelper.getcorrection(imu input)))
}

*/