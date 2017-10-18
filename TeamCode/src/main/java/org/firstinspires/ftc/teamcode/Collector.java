package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "Collector", group = "Pushbot")
public class Collector extends LinearOpMode
{

    //Declare Variables Used in Collector
    int stage = 0;
    Hardware robot = new Hardware();
    double liftPow = 0;
    double rotatePos = 0;
    double collectPos1 = 0;
    double collectPos2 = 0;

    //Define What The Robot Should Do When The Collector OpMode Is Run
    @Override
    public void runOpMode ()
    {
        robot.init(hardwareMap);
        waitForStart();

        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                switch (stage)
                {

                    case 0: //Initialize The Lift Motor, The Rotate Servo, And The Collect Servo
                        liftPow = 0;
                        collectPos1 = 0;
                        collectPos2 = 0;
                        rotatePos = 0;
                        stage = 1;
                        break;

                    case 1: //Move The Collect Servo To The Position In Which The Collector Can Grab A Cube
                        liftPow = 0;
                        if (robot.collect1.getPosition() < 0.5)
                        {
                            collectPos2 = 1;
                        }
                        else
                        {
                            collectPos1 = 1;
                        }
                        rotatePos = 0;
                        stage = 2;
                        break;

                    case 2: //Move The Lift Motor So That The Tower Is At The Height Where It Can Deposit The Cubes In The Higher Spots
                        liftPow = 1;
                        if (robot.collect1.getPosition() < 0.5)
                        {
                            collectPos2 = 1;
                        }
                        else
                        {
                            collectPos1 = 1;
                        }
                        rotatePos = 0;
                        stage = 3;
                        break;

                    case 3: //Move The Rotate Servo So That The Collector Is In The Inverse Position Compared To How It Started
                        liftPow = 1;
                        if (robot.collect1.getPosition() < 0.5)
                        {
                            collectPos2 = 1;
                        }
                        else
                        {
                            collectPos1 = 1;
                        }
                        if (robot.rotate.getPosition() < 0.5)
                        {
                            rotatePos = .5;
                        }
                        else
                        {
                            rotatePos = 0;
                        }
                        stage = 4;
                        break;

                    case 4: //Move The Lift Motor So That The Tower Is Back At The Original Height Where It Can Collect Another Cube
                        liftPow = -1;
                        if (robot.collect1.getPosition() < 0.5)
                        {
                            collectPos2 = 1;
                        }
                        else
                        {
                            collectPos1 = 1;
                        }
                        rotatePos = 1;
                        stage = 4;
                        break;

                }
                robot.lift.setPower(liftPow);
                robot.collect1.setPosition(collectPos1);
                robot.collect2.setPosition(collectPos2);
                robot.rotate.setPosition(rotatePos);
            }
        }
    }
}