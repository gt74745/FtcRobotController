package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Working Auton", group="Auton Finalization")
public class Auton extends LinearOpMode
{
    DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, flyWheelMotor;
	Servo pushServo;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public double leftFrontMotorPos, leftFrontDistanceTraveled = 0, deltaLeftFrontMotorPos = 0, previousLeftFrontMotorPos, leftFrontPower;
    public double rightFrontMotorPos, rightFrontDistanceTraveled = 0, deltaRightFrontMotorPos = 0, previousRightFrontMotorPos, rightFrontPower;
    public double leftBackMotorPos, leftBackDistanceTraveled = 0, deltaLeftBackMotorPos = 0, previousLeftBackMotorPos, leftBackPower;
    public double rightBackMotorPos, rightBackDistanceTraveled = 0, deltaRightBackMotorPos = 0, previousRightBackMotorPos, rightBackPower;
    double positiveDistanceTraveled, negativeDistanceTraveled;
    double distancePerTick = (2 * Math.PI * 48) / 537.6;
    int instruction = 1;
    double globalAngle = 0;
    double kP = 0.03;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeHardware();

        waitForStart();

        runtime.reset();

		flyWheelMotor.setPower(0.75);

        while (opModeIsActive())
        {
            // For debug
            telemetry.addData("1 positiveDistanceTraveled", positiveDistanceTraveled);
            telemetry.addData("2 negativeDistanceTraveled", negativeDistanceTraveled);
            telemetry.addData("3 leftFrontPower", leftFrontPower);
            telemetry.addData("4 rightFrontPower", rightFrontPower);
            telemetry.update();

            //Blue target zones
//            if (step == 1) move(760, 915); // Nearest zone
//            if (step == 1) move(1065, 915); // Middle zone
//            if (step == 1) move(1220, 1370); // Far zone

            //Red target zones
//            if (step == 1) move(915, 760); // Nearest zone
//            if (step == 1) move(915, 1065); // Middle zone
//            if (step == 1) move(1370, 1220); // Far zone

//            if (step == 2) move(915, 915); // Just Navigate

            if (instruction == 1) move(0, 760);
        }
    }

    private void initializeHardware()
    {
        //Init motors
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheelMotor = hardwareMap.dcMotor.get("flyWheelMotor");
        flyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		pushServo = hardwareMap.servo.get("pushServo");

        //Init IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;

        // Calculate the motor powers;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    void getMotorPositions()
    {
        // Update the positions of each individual motor
        leftFrontMotorPos = leftFrontMotor.getCurrentPosition();
        deltaLeftFrontMotorPos = distancePerTick * (leftFrontMotorPos - previousLeftFrontMotorPos);
        leftFrontDistanceTraveled += deltaLeftFrontMotorPos;
        previousLeftFrontMotorPos = leftFrontMotorPos;

        rightFrontMotorPos = rightFrontMotor.getCurrentPosition();
        deltaRightFrontMotorPos = distancePerTick * (rightFrontMotorPos - previousRightFrontMotorPos);
        rightFrontDistanceTraveled += deltaRightFrontMotorPos;
        previousRightFrontMotorPos = rightFrontMotorPos;

        leftBackMotorPos = leftBackMotor.getCurrentPosition();
        deltaLeftBackMotorPos = distancePerTick * (leftBackMotorPos - previousLeftBackMotorPos);
        leftBackDistanceTraveled += deltaLeftBackMotorPos;
        previousLeftBackMotorPos = leftBackMotorPos;

        rightBackMotorPos = rightBackMotor.getCurrentPosition();
        deltaRightBackMotorPos = distancePerTick * (rightBackMotorPos - previousRightBackMotorPos);
        rightBackDistanceTraveled += deltaRightBackMotorPos;
        previousRightBackMotorPos = rightBackMotorPos;
    }

    void move(double posTarget, double negTarget)
    {
        getMotorPositions();

        positiveDistanceTraveled = (leftFrontDistanceTraveled + rightBackDistanceTraveled) / 2;
        negativeDistanceTraveled = (leftBackDistanceTraveled + rightFrontDistanceTraveled) / 2;

        double positiveError = (posTarget) - positiveDistanceTraveled;
        double negativeError = (negTarget) - negativeDistanceTraveled;

        // Calculate the motor powers
        leftFrontPower = (kP * positiveError);
        rightFrontPower = (kP * negativeError);
        leftBackPower = (kP * negativeError);
        rightBackPower = (kP * positiveError);

        // Stop if we reach the target position
        if (positiveError > -10 && positiveError < 10)
        {
            leftFrontPower = 0;
            rightBackPower = 0;
        }

        if (negativeError > -10 && negativeError < 10)
        {
            rightFrontPower = 0;
            leftBackPower = 0;
        }

        if (leftFrontPower > 0.25) { leftFrontPower = 0.25; }
        if (rightFrontPower > 0.25) { rightFrontPower = 0.25; }
        if (leftBackPower > 0.25) { leftBackPower = 0.25; }
        if (rightBackPower > 0.25) { rightBackPower = 0.25; }

        // Apply motor power
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    void turn(double degrees)
    {
        // Get the current orientation of the bot
        double angle = getAngle();

        degrees -= 13; //Account for +13 degrees gltich.

        // Make sure we're not at the target. If not, move. If so, stop.
        if (angle < degrees) {
            leftFrontPower = (degrees - angle);
            rightFrontPower = -(degrees - angle);
            leftBackPower = (degrees - angle);
            rightBackPower = -(degrees - angle);
        } else
        {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
            instruction++;
        }

        // Limit motor power so we don't lose accuracy
        if (leftFrontPower > 0.35) { leftFrontPower = 0.35; }
        if (rightFrontPower < -0.35) { rightFrontPower = -0.35; }
        if (leftBackPower > 0.35) { leftBackPower = 0.35; }
        if (rightBackPower < -0.35) { rightBackPower = -0.35; }

        // Apply motor power
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    private double getAngle()
    {
        // Get the orientation from the gyroscope
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // The number we get from the gyro has no limit. We can technically use it like this but
        // that would be messy.
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        // This value is now the orientation of the bot represented by a degree between -180 and 180
        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

	public void loadRing() {
		pushServo.setPosition(0.25);
	}

	public void resetServo() {
		pushServo.setPosition(0);
	}

	public void launchRing() {}	
}

