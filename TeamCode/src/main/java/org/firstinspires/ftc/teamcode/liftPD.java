package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Pureawesomeness on 11/8/2016.
 */
public class liftPD
{
    final static double Kp = 0.1;
    final static double Kd = 0.01;
    public liftPD()
    {
    }

    // Use a PD to determine a motor command, which has a range of -1 to 1 (-1=rev; 0=stop; 1=fwd)
    public static double getMotorCmd (double targetPos, double encoder, double maxPower)
    {

        double motorCmd;
        double error = targetPos - encoder;
        motorCmd = (Kp * error);
        motorCmd = Range.clip(motorCmd, -maxPower, maxPower);
        //        motorCmd = Range.clip(motorCmd, -1.0, 1.0);

        return motorCmd;
    }



}