package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ControllerHelper;

@TeleOp (name = "DriveMode")
public class Drive extends LinearOpMode {

    //Motor Variables
    private DcMotor lfMotor, lbMotor, rfMotor, rbMotor;
    private DcMotor[] motors;

    //IMU Variables
    private BNO055IMU imu;
    private Orientation lastOrientation;

    //Values
    private ControllerHelper.MotorPower motorPower; //Motor power values.
    private ControllerHelper.ControllerValues controllerValues; //Controller values
    private double lfMotorPos, lastLfMotorPos, deltaLfMotorPos, lfMotorDist; //Using lf motor pos for debug stats.
    private double correction, target, globalHeading; //Make sure robot corrects itself to hit target.


    //Utility
    private ControllerHelper controllerHelper;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        log(new String[] {"Status"}, new String[] {"OpMode is running."});

        sleep(1000);

        movementLoop();
    }

    private void movementLoop()
    {
        while(opModeIsActive())
        {
            correction = getCorrection();
            final double angle = getAngle();

            log(new String[] {
                    "[1] IMU Heading",
                    "[2] Global Heading",
                    "[3] Correction Margin",
                    "[4] Angle",
                    "[5] X",
                    "[6] Y",
                    "[7] TPC",
                    "[8] DTC",
                    "[9] Distance Traveled"
            }, new String[] {
                    fieldString(lastOrientation.firstAngle),
                    fieldString(globalHeading),
                    fieldString(correction),
                    fieldString(angle),
                    fieldString(controllerValues.horizontal),
                    fieldString(controllerValues.vertical),
                    fieldString(lfMotorPos),
                    fieldString(deltaLfMotorPos),
                    fieldString(lfMotorDist)
            });

            motorPower = controllerHelper.getMotorPower(angle); //Calculate power
            setMotorPower(isTurning());
            calculateMotorStats();
        }

        for (final DcMotor motor : motors) //Final motor commands
        {
            motor.setPower(0); //Power begone.
        }
    }

    //Set all necessary hardware variables
    private void initializeHardware()
    {
        log(new String[] {"Status"}, new String[] {"Initializing"});

        try {
            //Controller Helper
            controllerHelper = new ControllerHelper(this);

            //Motors
            lfMotor = hardwareMap.dcMotor.get("leftFrontMotor");
            lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lbMotor = hardwareMap.dcMotor.get("leftBackMotor");
            lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rfMotor = hardwareMap.dcMotor.get("rightFrontMotor");
            rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rbMotor = hardwareMap.dcMotor.get("rightBackMotor");
            rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motors = new DcMotor[] {lfMotor, lbMotor, rfMotor, rbMotor};

            //IMU
            final BNO055IMU.Parameters params = new BNO055IMU.Parameters();

            params.mode = BNO055IMU.SensorMode.IMU;
            params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            params.loggingEnabled = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(params);

            //Wait for IMU to calibrate
            while (!isStopRequested() && !imu.isGyroCalibrated())
            {
                sleep(50);
                idle();
            }

            log(new String[] {"Status"}, new String[] {"Successfully Initialized and calibrated. Waiting for manual control."});

        } catch (final Exception ex)
        {
            log(new String[] {"Initializing error"}, new String[] {ex.getMessage()});
            stop(); //Probably an issue if something went wrong here.
        }
    }

    private void setMotorPower(final boolean turning)
    {
        if (turning)
        {
            motorPower.lf_rbPwr = 0.5 * controllerValues.rotation;
            motorPower.rf_lbPwr = 0.5 * controllerValues.rotation;

            lfMotor.setPower(-0.7 * motorPower.lf_rbPwr);
            rbMotor.setPower(-0.7 * motorPower.lf_rbPwr);
            lbMotor.setPower(-0.7 * motorPower.rf_lbPwr);
            rfMotor.setPower(-0.7 * motorPower.rf_lbPwr);

            target = globalHeading;
        } else
        {
            lfMotor.setPower(-0.75 * motorPower.lf_rbPwr + correction);
            rbMotor.setPower(-0.75 * motorPower.lf_rbPwr + correction);
            lbMotor.setPower(-0.75 * motorPower.rf_lbPwr + correction);
            rfMotor.setPower(-0.75 * motorPower.rf_lbPwr + correction);
        }
    }

    //Calculate error so we can make up for margin.
    private double getCorrection()
    {
        final Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastOrientation.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalHeading += deltaAngle;

        lastOrientation = orientation;

        double correction;
        final double gain = 0.03;

        if (globalHeading == target)
            correction = 0;
        else
            correction = -(globalHeading - target);

        correction = correction * gain;

        return correction;
    }

    private double getAngle()
    {
        double angle;

        if (controllerValues.horizontal > 0)
        {
            angle = (Math.atan(-controllerValues.vertical / controllerValues.horizontal) - Math.PI / 4);
        } else
        {
            angle = (Math.atan(-controllerValues.vertical / controllerValues.horizontal) + Math.PI - Math.PI / 4);
        }
        angle = angle - ((lastOrientation.firstAngle) / Constants.degreesPerRadian);

        controllerValues = controllerHelper.getControllerValues();

        return angle;
    }

    private boolean isTurning()
    {
        final ControllerHelper.ControllerValues values = controllerHelper.getControllerValues();

        if (values.rotation != 0 && values.vertical == 0 && values.horizontal == 0)
            return true;
        else
            return false;
    }

    //Cleaner than telemetry.addData
    private void log(final String[] key, final String[] data)
    {
        for (int i = 0; i < key.length; i++)
        {
            telemetry.addData(key[i], data[i]);
        }

        telemetry.update();
    }

    //Get motor positions and distances
    private void calculateMotorStats()
    {
        lfMotorPos = lfMotor.getCurrentPosition();
        deltaLfMotorPos = Constants.distancePerTick * (lfMotorPos - lastLfMotorPos);
        lfMotorDist += deltaLfMotorPos;
        lastLfMotorPos = lfMotorPos;
    }

    //Utility
    private String fieldString(final Object field)
    {
        return String.valueOf(field);
    }
}
