package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Hanna on 9/30/2017.
 */

public class Gripper extends PinkNavigate{
    int Groped = 0;//make boolean????
    int Extend = 0;

if( /*button to open grip*/)
    {
        Groped = 10;
    }
else if (/*button to close*/)
    {
        Groped = 20;
    }


    switch (Groped)
    {
        case 10:
            robot.collect.setPosition(0.0);
            //grip is open
            break;
         case 20:
            robot.collect.setPosition(1.0);
        break;
}


switch(Extend) {
        case 10:
        robot.Extend.setPower(1.0);
        break;

        case 20:
        robot.Extend.setPower(0.0)
        break;

        case 30:
        robot.Extend.setPower(-1.0)
        break;

        }