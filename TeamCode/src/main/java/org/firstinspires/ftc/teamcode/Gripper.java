package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Gripper", group = "Pushbot")
public class Gripper extends LinearOpMode {

    //Declare Variables Used In Gripper
    double              relicLiftPos;
    double              extendPow;
    double              grabPos;
    double              rotatePos;
    int         stage   = 0;
    Hardware    robot   = new Hardware();

    //Define What The Robot Should Do When The Gripper OpMode Is Run
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                switch (stage) {

                    case 0: //Initialize The Lift Relic Motor, The Extend Motor, The Grab Servo, And The Rotate Relic Servo
                        relicLiftPos = 0;
                        extendPow    = 0;
                        grabPos      = 0;
                        rotatePos    = 0;
                        stage = 1;
                        break;

                    case 1: //Move The Rotate Relic Servo To The Position Where The Arm Is Perpendicular To The Floor
                        relicLiftPos = 0;
                        extendPow    = 0;
                        grabPos      = 0;
                        rotatePos    = .25;
                        stage = 2;
                        break;

                    case 2: //Move The Lift Relic Motor To the Postition Where The Arm Is At The Level Where It Could Grab The Relic
                        relicLiftPos = 1;
                        extendPow    = 0;
                        grabPos      = 0;
                        rotatePos    = .25;
                        stage = 3;
                        break;

                    case 3: //Move The Grab Servo To The Position Where The Claw Is Open So It Could Grab The Relic
                        relicLiftPos = 1;
                        extendPow    = 0;
                        grabPos      = 1;
                        rotatePos    = 0.25;
                        stage = 4;
                        break;

                    case 4: //Move The Grab Servo To The Position Where The Claw Can Grab The Relic
                        relicLiftPos = 1;
                        extendPow    = 0;
                        grabPos      = 0;
                        rotatePos    = .25;
                        stage = 5;
                        break;

                    case 5: //Move The Lift Relic Motor To The Postition Where The Arm Is At The Level Where The Relic Can Be Extended Past The Wall
                        relicLiftPos = -1;
                        extendPow    = 0;
                        grabPos      = 0;
                        rotatePos    = .25;
                        stage = 6;
                        break;

                    case 6: //Move The Extend Motor To The Position Where The Arm Is Reaching Out So The Relic Can Score In The Highest Scoring Point
                        relicLiftPos = -1;
                        extendPow    = 1;
                        grabPos      = 0;
                        rotatePos    = .25;
                        stage = 7;
                        break;

                    case 7: //Move The Lift Relic Motor To The Position Where The Relic Can Be Released Without Falling Over
                        relicLiftPos = 1;
                        extendPow    = 1;
                        grabPos      = 0;
                        rotatePos    = .25;
                        stage = 8;
                        break;

                    case 8: //Move The Grab Servo To The Position Where The Relic Is Released From The Claw
                        relicLiftPos = 1;
                        extendPow    = 1;
                        grabPos      = 1;
                        rotatePos    = .25;
                        break;
                }
                robot.liftRelic.setPower(relicLiftPos);
                robot.extend.setPower(extendPow);
                robot.grab.setPosition(grabPos);
                robot.rotaterelic.setPosition(rotatePos);
            }
        }
    }
}