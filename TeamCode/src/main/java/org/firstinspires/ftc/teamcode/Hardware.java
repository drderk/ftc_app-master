package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Hardware {

    //-----------------------------------------Motors-----------------------------------------//
    public DcMotor craneExtend  = null;
    public DcMotor craneRotate  = null;
    public DcMotor armRotate    = null;
    public DcMotor leftDrive    = null;
    public DcMotor rightDrive   = null;
    public DcMotor light1       = null;
    public DcMotor light2       = null;

    //-----------------------------------------Servos-----------------------------------------//
    public Servo collectorFinger1   = null;
    public Servo collectorFinger2   = null;
    public Servo collectorFinger3   = null;
    public Servo collectorFinger4   = null;
    public Servo collectorRotate    = null;
    public Servo craneWrist         = null;
    public Servo craneClaw          = null;
    public Servo flickerArm         = null;
    public Servo flickerFinger      = null;

    //-----------------------------------------Sensors----------------------------------------//
    public ColorSensor    colorSensor       = null;
    public DistanceSensor distanceSensor    = null;

    //----------------------------------Local OpMode Members----------------------------------//
    HardwareMap hwMap = null;

    //---------------------Initialization of Standard Hardware Interfaces---------------------//
    public void init (HardwareMap ahwMap) {

        //--------------------Saving the Reference to the Hardware Map--------------------//
        hwMap = ahwMap;

        //-------------------------------------Motors-------------------------------------//
        leftDrive   = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hwMap.get(DcMotor.class, "rightDrive");
        craneExtend = hwMap.get(DcMotor.class, "craneExtend");
        craneRotate = hwMap.get(DcMotor.class, "craneRotate");
        armRotate   = hwMap.get(DcMotor.class, "armRotate");
        light1      = hwMap.get(DcMotor.class, "light1");
        light2      = hwMap.get(DcMotor.class, "light2");

        //-------------------------------------Servos-------------------------------------//
        collectorFinger1    = hwMap.get(Servo.class, "collectorFinger1");
        collectorFinger2    = hwMap.get(Servo.class, "collectorFinger2");
        collectorFinger3    = hwMap.get(Servo.class, "collectorFinger3");
        collectorFinger4    = hwMap.get(Servo.class, "collectorFinger4");
        collectorRotate     = hwMap.get(Servo.class, "collectorRotate");
        craneWrist          = hwMap.get(Servo.class, "craneWrist");
        craneClaw           = hwMap.get(Servo.class, "craneClaw");
        flickerArm          = hwMap.get(Servo.class, "flickerArm");
        flickerFinger       = hwMap.get(Servo.class, "flickerFinger");

        //-------------------------------------Sensors------------------------------------//
        colorSensor     = hwMap.get(ColorSensor.class, "colorSensor");
        distanceSensor  = hwMap.get(DistanceSensor.class, "colorSensor");

        //-------------------------------Motor Configuration------------------------------//
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        craneRotate.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armRotate.setPower(0);
    }
}