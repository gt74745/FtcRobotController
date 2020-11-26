package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerHelper {

    private final OpMode opMode;

    public ControllerHelper(final OpMode opmode)
    {
        opMode = opmode;
    }

    //Get controller axes for drive control.
    public ControllerValues getControllerValues()
    {
        final ControllerValues values = new ControllerValues();

        if (values.horizontal != 0)
            values.horizontal = opMode.gamepad1.left_stick_x;

        if (values.vertical != 0)
            values.vertical = opMode.gamepad1.left_stick_y;

        if (values.rotation != 0)
            values.rotation = opMode.gamepad1.right_stick_x;

        return values;
    }

    //Calculate position of joystick relative to its origin and translate to drive.
    public MotorPower getMotorPower(final double angle)
    {
        final MotorPower power = new MotorPower();
        final double dist = getDistanceFromOrigin();

        if (angle != 0)
        {
            power.lf_rbPwr = 0.8 * (dist * Math.cos(angle));
            power.rf_lbPwr = 0.8 * (dist * Math.sin(angle));

            return power;
        }

        power.rf_lbPwr = 0.8 * (dist * Math.sin(angle));
        power.lf_rbPwr = power.rf_lbPwr;

        return  power;
    }

    //Calculate joystick distance from its resting position.
    public double getDistanceFromOrigin()
    {
        final ControllerValues values = getControllerValues();
        return Math.sqrt(
                Math.pow(values.horizontal, 2) + Math.pow(values.vertical, 2));
    }

    public class MotorPower
    {
        /**
         * Right-Front and Left-Back Motor Power
         */
        public double rf_lbPwr;
        /**
         * Left-Front and Right-Back Motor Power
         */
        public double lf_rbPwr;
    }

    public class ControllerValues
    {
        //If this was 0, things will break..
        public double vertical = 0.0000001;
        public double horizontal = 0.0000001;
        public double rotation = 0.0000001;
    }


}
