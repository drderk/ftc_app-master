package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDIO;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorHTColor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRIrSeeker;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */
public class Hardware
{
    //Motors

    public DcMotor craneExtend = null;
    public DcMotor craneRotate = null;
    public DcMotor armRotate = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor light1 = null;
    public DcMotor light2 = null;

    public Servo collectorRotate  = null;
    public Servo collectorFinger1 = null;
    public Servo collectorFinger2 = null;
    public Servo collectorFinger  = null;
    public Servo craneWrist = null;
    public Servo craneClaw = null;
    public Servo flickerArm = null;
    public Servo flickerFinger = null;

    //    public Rev colorSensor = null;
    public ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;

    public SensorDIO breakBeam = null;
    public SensorBNO055IMU compass = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware ()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init (HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //  Motors
        leftDrive = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        craneExtend = hwMap.get(DcMotor.class, "craneExtend");
        craneRotate = hwMap.get(DcMotor.class, "craneRotate");
        armRotate = hwMap.get(DcMotor.class, "armRotate");
        light1 = hwMap.get(DcMotor.class, "light1");
        light2 = hwMap.get(DcMotor.class, "light2");

        // Sensors
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hwMap.get(DistanceSensor.class, "colorSensor");

        //Servos
        collectorFinger1 = hwMap.get(Servo.class, "collectorFinger1");
        collectorFinger2 = hwMap.get(Servo.class, "collectorFinger2");
//        collectorFinger  = hwMap.get(Servo.class, "collectorFinger"); //This is for the new collector
        collectorRotate = hwMap.get(Servo.class, "collectorRotate");
        craneWrist = hwMap.get(Servo.class, "craneWrist");
        craneClaw = hwMap.get(Servo.class, "craneClaw");
        flickerArm = hwMap.get(Servo.class, "flickerArm");
        flickerFinger = hwMap.get(Servo.class, "flickerFinger");

        // *** Motor Configuration
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        light1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        light2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        craneExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        craneRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        light1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        light2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armRotate.setDirection(DcMotor.Direction.REVERSE);
        craneExtend.setDirection(DcMotor.Direction.REVERSE);
        craneRotate.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armRotate.setPower(0);
    }
//
//    /***
//     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
//     * periodic tick.  This is used to compensate for varying processing times for each cycle.
//     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
//     *
//     * @param periodMs  Length of wait cycle in mSec.
//     * @throws InterruptedException
//     */
//    public void waitForTick (long periodMs) throws InterruptedException
//    {
//
//        long remaining = periodMs - (long) period.milliseconds();
//
//        // sleep for the remaining portion of the regular cycle period.
//        if (remaining > 0)
//            Thread.sleep(remaining);
//
//        // Reset the cycle clock for the next pass.
//        period.reset();
//    }
}

