package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Perfected Driver", group="XDrive")
public class DriverControl extends LinearOpMode {
    DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, flyWheelMotor, pickupMotor;
    Servo pushServo;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction, horizontal, vertical, joystickDistanceFromOrigin, angle, pwr, pwr2, target, leftFrontMotorPos;
    // Circumference of the wheels divided by ticks per revolution
    double distancePerTick = (2 * Math.PI * 48) / 537.6;
    double previousLeftFrontMotorPos = 0;
    double deltaLeftFrontMotorPos = 0;
    double leftFrontDistanceTraveled = 0;
    boolean isTurning;

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        /*while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }*/

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

	    flyWheelMotor.setPower(0.75); //
        pickupMotor.setPower(-0.25);

        sleep(1000);

        while (opModeIsActive()) {
            correction = getCorrectionValue();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 angle", angle);
            telemetry.addData("5 x", horizontal);
            telemetry.addData("6 y", vertical);
            telemetry.addData("7 ticks per cycle", leftFrontMotorPos);
            telemetry.addData("8 distance traveled per cycle", deltaLeftFrontMotorPos);
            telemetry.addData("9 total distance traveled", leftFrontDistanceTraveled);
            telemetry.update();

            vertical = gamepad1.left_stick_y;
            double turning = gamepad1.right_stick_x;
	    
	        boolean triggerIsPressed = gamepad1.a;
	        boolean ringLoaded = false;

            // Calculate distance between the current joystick position and the idle position
            joystickDistanceFromOrigin = Math.sqrt(Math.pow(horizontal, 2) + Math.pow(vertical, 2));
            //position = Math.abs((Math.sqrt(2)) * (leftFrontMotor.getCurrentPosition()) / (537.6) * 96 * Math.PI);

            // Call necessary methods
            calcPower();
            checkIfTurning();
            getAngle();

            if (isTurning) {
                pwr = 0.5 * turning;
                pwr2 = 0.5 * turning;
                leftFrontMotor.setPower(-0.7 * pwr);
                rightFrontMotor.setPower(-0.7 * pwr2);
                leftBackMotor.setPower(-0.7 * pwr2);
                rightBackMotor.setPower(-0.7 * pwr);
                //orientation += (newTurn - oldTurn);
                target = globalAngle;
            }

            // Check the power values and correct them
            // it would be such an L pwr = checkDirection(pwr);
            // pwr2 = checkDirection(pwr2);

            // Give calculated power to the motors
            if (!isTurning) {
                leftFrontMotor.setPower(-0.75 * pwr + correction);
                rightFrontMotor.setPower(0.75 * pwr2 + correction);
                leftBackMotor.setPower(-0.75 * pwr2 + correction);
                rightBackMotor.setPower(0.75 * pwr + correction);
            }

	        //Load and shoot rings.
    	    if (triggerIsPressed) {
        		loadRing();
        		sleep(250);
    	    }

    	    if (!triggerIsPressed) {
    	    	resetServo();
    	    }

            leftFrontMotorPos = leftFrontMotor.getCurrentPosition();
            deltaLeftFrontMotorPos = distancePerTick * (leftFrontMotorPos - previousLeftFrontMotorPos);
            leftFrontDistanceTraveled += deltaLeftFrontMotorPos;
            previousLeftFrontMotorPos = leftFrontMotorPos;
        }

        // turn the motors off.
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    private void initializeHardware()
    {
        //Init motors
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	    flyWheelMotor = hardwareMap.dcMotor.get("flyWheelMotor");
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pickupMotor = hardwareMap.dcMotor.get("pickupMotor");
        pickupMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


	    pushServo = hardwareMap.servo.get("pushServo");
	    resetServo();

        //Init IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private double getCorrectionValue() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        double correction, gain = .03;

        if (globalAngle == target)
            correction = 0;
        else
            correction = -(globalAngle - target);

        correction = correction * gain;

        return correction;
    }

    public void getAngle() {
        if (horizontal > 0) {
            angle = (Math.atan(-vertical / horizontal) - Math.PI / 4);

            // FOR FUTURE REFERENCE this ridiculous number is literally just degrees per radian.
            angle = angle - ((lastAngles.firstAngle) / 57.29577951);
        }
        if (horizontal < 0) {
            angle = (Math.atan(-vertical / horizontal) + Math.PI - Math.PI / 4);
            angle = angle - ((lastAngles.firstAngle) / 57.29577951);
        }
        if (gamepad1.left_stick_x == 0) {
            horizontal = 0.00000000000001;
        } else {
            horizontal = gamepad1.left_stick_x;
        }
    }

    public void checkIfTurning() {
        if (gamepad1.right_stick_x != 0) {
            isTurning = true;
        }
        if ((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0)) {
            isTurning = false;
        }
    }

    public void calcPower() {
        // Calculate motor power
        if (angle == 0) {
            pwr2 = 0.8 * (joystickDistanceFromOrigin * Math.sin(angle));
            pwr = pwr2;
        } else {
            pwr = 0.8 * (joystickDistanceFromOrigin * Math.cos(angle));
            pwr2 = 0.8 * (joystickDistanceFromOrigin * Math.sin(angle));
        }
    }

    public void loadRing() {
	// actuate the piston forward push the bottom ring into the flywheel.
	pushServo.setPosition(1);
    }

    public void resetServo() {
	// reset the position of the servo.
	pushServo.setPosition(0.6);
    }
}
