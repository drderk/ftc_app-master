package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Collector", group = "Pushbot")
public class Collector extends LinearOpMode {

    //Declare Variables Used in Collector
    int         stage   = 0;
    Hardware    robot   = new Hardware();

    //Define What The Robot Should Do When The Collector OpMode Is Run
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        if(opModeIsActive()){
            while (opModeIsActive()) {
                switch (stage) {

                    case 0: //Initialize The Lift Motor, The Rotate Servo, And The Collect Servo
                        robot.lift.setPower(0);
                        robot.collect.setPosition(0);
                        robot.rotate.setPosition(0);
                        stage = 1;
                        break;

                    case 1: //Move The Collect Servo To The Position In Which The Collector Can Grab A Cube
                        robot.collect.setPosition(1);
                        stage = 2;
                        break;

                    case 2: //Move The Lift Motor So That The Tower Is At The Height Where It Can Deposit The Cubes In The Higher Spots
                        robot.lift.setPower(1);
                        stage = 3;
                        break;

                    case 3: //Move The Rotate Servo So That The Collector Is In The Inverse Position Compared To How It Started
                        robot.rotate.setPosition(0.5);
                        stage = 4;
                        break;

                    case 4: //Move The Lift Motor So That The Tower Is Back At The Original Height Where It Can Collect Another Cube
                        robot.lift.setPower(-1);
                        break;
                }
            }
        }
    }
}