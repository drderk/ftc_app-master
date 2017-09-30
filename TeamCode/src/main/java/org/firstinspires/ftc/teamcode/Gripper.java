package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Hanna on 9/30/2017.
 */

public class Gripper extends PinkNavigate{
    public int Groped = 0;
    public int Extend = 0;

if( /*button to open grip*/== true)
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
            robot.collect.setPower(0.0);
            //grip is open
            break;
    }
    case 20:
            robot.collect.setPower(1.0);
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