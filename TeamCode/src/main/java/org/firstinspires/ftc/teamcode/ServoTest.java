package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ServoTest extends LinearOpMode {
	Servo servo1;

	@Override
	public void runOpMode(){
		servo1 = hardwareMap.servo.get("servo1");
		servo1.setPosition(0);
		waitForStart();
		while(opModeIsActive()){
			servo1.setPosition(1);
		}
	}
}
