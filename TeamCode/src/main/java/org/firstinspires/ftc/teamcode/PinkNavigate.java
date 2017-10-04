package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware;

public class PinkNavigate
{
    static Hardware robot = new Hardware();


    static final double COUNTS_PER_INCH = 1000;  // Base travel
    static final double POSITION_THRESHOLD = 10.0;   // Counts
    static final double ANGLE_THRESHOLD = 5.0;     // Degrees

    // Tank drive two wheels to target positions in inches.
    // Returns true when both arrive at the target.
    public static boolean driveToPos(double targetPos, double targetAngle, int currentAngle,
    double linearVelocity, double angularVelocity, double maxPower)
    {
        double motorCmd = 0;
        double targetPosCounts = targetPos * COUNTS_PER_INCH;
        double leftMotorCmd, rightMotorCmd;
        double leftWheelPos = robot.rightDrive.getCurrentPosition();
        double rightWheelPos = robot.leftDrive.getCurrentPosition();
        double angleErrorDegrees = targetAngle - currentAngle;
        double currentPosCounts = (leftWheelPos + rightWheelPos)/2.0;
        double angleOffset;
        double linearError = targetPosCounts - currentPosCounts;
        double angularError = targetAngle - currentAngle;

        // Determine the baseline motor speed command
        motorCmd = Range.clip(motorCmd, -0.5, 0.5);

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

        robot.leftDrive.setPower(leftMotorCmd);
        robot.rightDrive.setPower(rightMotorCmd);
        if((Math.abs(linearError)<POSITION_THRESHOLD)&&(Math.abs(angleErrorDegrees)<ANGLE_THRESHOLD))
        {
             return false;
        } else
        {
            return true;
        }
    }

    public static void stopBase()
    {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public static void runUsingEncoders()
    {
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void runWithoutEncoders()
    {
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static boolean resetBasePosition()
    {
        boolean resetStatus = false;

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if ((robot.leftDrive.getCurrentPosition()==0)&&(robot.rightDrive.getCurrentPosition()==0))
        {
            resetStatus = true;
        }
        return resetStatus;
    }

    public static double getBasePosition()
    {
        return ((robot.leftDrive.getCurrentPosition() + robot.rightDrive.getCurrentPosition())/2.0);
    }

}
