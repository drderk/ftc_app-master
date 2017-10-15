package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware;

public class PinkNavigate
{
    Hardware robot;

    static final double COUNTS_PER_INCH = 49.5;  // Base travel
    static final double POSITION_THRESHOLD = 30.0;   // Counts
    static final double ANGLE_THRESHOLD = 10.0;     // Degrees
    double leftMotorCmd, rightMotorCmd;
    // Tank drive two wheels to target positions in inches.
    // Returns true when both arrive at the target.
    public boolean driveToPos(double targetPos, double targetAngle, int currentAngle, double leftEnc, double rightEnc,
    double linearVelocity, double angularVelocity, double maxPower)
    {
        double targetPosCounts = targetPos * COUNTS_PER_INCH;
        double leftWheelPos = leftEnc;
        double rightWheelPos = rightEnc;
        double angleErrorDegrees = targetAngle - currentAngle;
        double currentPosCounts = (leftWheelPos + rightWheelPos)/2.0;
        double angleOffset;
        double linearError = targetPosCounts - currentPosCounts;
        double angularError = targetAngle - currentAngle;
        double motorCmd = PinkPD.getMotorCmd(0.1,0.001,linearError,linearVelocity);

        // Determine the baseline motor speed command
        motorCmd = Range.clip(motorCmd, -0.6, 0.6);

        // Determine and add the angle offset
        angleOffset = PinkPD.getMotorCmd(0.02, 0.02, angularError, angularVelocity);
        leftMotorCmd = motorCmd + angleOffset;
        rightMotorCmd = motorCmd - angleOffset;
        leftMotorCmd = Range.clip(leftMotorCmd, -1.0, 1.0);
        rightMotorCmd = Range.clip(rightMotorCmd, -1.0, 1.0);

        // Scale the motor commands back to account for the MC windup problem
        // (if the motor cant keep up with the command, error builds up)
        leftMotorCmd *= maxPower;
        rightMotorCmd *= maxPower;

        // If navigated to position
        if((Math.abs(linearError)<POSITION_THRESHOLD)&&(Math.abs(angleErrorDegrees)<ANGLE_THRESHOLD))
        {
             return true;
        } else
        {
            return false;
        }
    }

    public void stopBase()
    {
        leftMotorCmd = 0;
        rightMotorCmd = 0;
    }

    public double getRightCMD(){
        return rightMotorCmd;
    }
    public double getLeftCMD(){
        return leftMotorCmd;
    }
}
