package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

public class PinkNavigate
{
    static final double COUNTS_PER_INCH = 49.8; // Counts
    static final double POSITION_THRESHOLD = 30.0;   // Base travel
    static final double ANGLE_THRESHOLD = 10.0;     // Degrees
    Hardware robot;
    static double leftMotorCmd, rightMotorCmd;

    // Tank drive two wheels to target positions in inches.
    // Returns true when both arrive at the target.
    public static boolean driveToPos (double targetPosInches, double targetAngleDeg, double currentBasePosCounts, int currentAngleDeg,
                               double linearSpeedCounts, double maxPower)
    {
        double angleErrorDegrees = targetAngleDeg - currentAngleDeg;
        double currentPosInches = (currentBasePosCounts / COUNTS_PER_INCH);
        double linearSpeedInches = linearSpeedCounts / COUNTS_PER_INCH;
        double angleOffset;
        double linearError = targetPosInches - currentPosInches;
        double angularError = targetAngleDeg - currentAngleDeg;
        double motorCmd = PinkPD.getMotorCmd(0.05, 0.2, linearError, linearSpeedInches);

        // Determine the baseline motor speed command, but limit it to leave room for the turn offset
        motorCmd = Range.clip(motorCmd, -0.6, 0.6);

        // Determine and add the angle offset
        angleOffset = PinkPD.getMotorCmd(0.05, 0.001, angularError, 0);
        leftMotorCmd = motorCmd - angleOffset;
        rightMotorCmd = motorCmd + angleOffset;
        leftMotorCmd = Range.clip(leftMotorCmd, -1.0, 1.0);
        rightMotorCmd = Range.clip(rightMotorCmd, -1.0, 1.0);

        // Limit the max motor command for gentle motion
        leftMotorCmd = Range.clip(leftMotorCmd, -maxPower, maxPower );
        rightMotorCmd = Range.clip(rightMotorCmd, -maxPower, maxPower );

        // If navigated to position
        if ((Math.abs(linearError) < POSITION_THRESHOLD) && (Math.abs(angleErrorDegrees) < ANGLE_THRESHOLD))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void stopBase ()
    {
        leftMotorCmd = 0;
        rightMotorCmd = 0;
    }

    public double getRightCMD ()
    {
        return rightMotorCmd;
    }

    public double getLeftCMD ()
    {
        return leftMotorCmd;
    }
}
