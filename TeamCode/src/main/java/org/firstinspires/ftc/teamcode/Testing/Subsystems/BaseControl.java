package org.firstinspires.ftc.teamcode.Testing.Subsystems;


public class BaseControl {

    static double leftWheelsMotorCmd;
    static double rightWheelsMotorCmd;


    public static double leftDriveFullSpeed(double leftJoystick1) {
        leftWheelsMotorCmd = leftJoystick1 * 1.0;
        return leftWheelsMotorCmd;
    }

    public static double rightDriveFullSpeed(double rightJoystick1) {
        rightWheelsMotorCmd = rightJoystick1 * 1.0;
        return rightWheelsMotorCmd;
    }

    public static double leftDriveHalfSpeed(double leftJoystick1) {
        leftWheelsMotorCmd = leftJoystick1 * 0.5;
        return leftWheelsMotorCmd;
    }

    public static double rightDriveHalfSpeed(double rightJoystick1) {
        rightWheelsMotorCmd = rightJoystick1 * 0.5;
        return rightWheelsMotorCmd;
    }
}