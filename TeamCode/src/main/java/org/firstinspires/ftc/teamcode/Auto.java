/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous (name = "AutoTestDrive")
@Disabled
public class Auto extends OpMode
{

    /* Declare OpMode members. */
    Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware
    BNO055IMU imu;
    int currentBaseAngle;
    int stage = 0;
    double leftWheelMotorCmd, rightWheelMotorCmd, armMotorCmd;
    double collectorArmTargetPos;
    double flickerArmTargetPos, flickerFingerTargetPos;
    double collectorFinger1TargetPos, collectorFinger2TargetPos, collectorRotateTargetPos;
    double targetBasePos, targetBaseAngle;
    double baseScorePos, baseScoreAngle;
    boolean jewelFound = false;
    boolean blueAlliance;        // Selected alliance color
    boolean cornerStartingPos;   // Corner or middle starting position
    double markedTime;
    boolean ourJewelIsTheFrontOne;
    double leftWheelPos = 0, rightWheelPos = 0;
    double currentBasePos, previousBasePos;
    double linearBaseSpeed = 0;
    double columnOffset = 0;
    double craneRotatePos = 0;
    double craneExtendPos = 0;
    double collectorArmPos = 0, collectorArmPreviousPos = 0, armSpeed = 0;
    RelicRecoveryVuMark image = null;

    VuMarks camera = new VuMarks();
    RelicRecoveryVuMark picturePos;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init ()
    {
        boolean robotAutoConfigured = false;
         /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        blueAlliance = true;
        cornerStartingPos = true;

        // Wait for the game to start (driver presses PLAY).
        telemetry.addData("Status", "Waiting for start");    //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop ()
    {
        if (gamepad1.x){
            blueAlliance = true;
        }
        else if (gamepad1.b){
            blueAlliance = false;
        }
        if (gamepad1.y){
            cornerStartingPos = true;
        }
        else if (gamepad1.a){
            cornerStartingPos = false;
        }
        if (blueAlliance){
            telemetry.addData("Alliance Color", "BLUE");
        } else
        {
            telemetry.addData("Alliance Color", "RED");
        }
        if (cornerStartingPos){
            telemetry.addData("Starting Pos  ", "CORNER");
        } else
        {
            telemetry.addData("Starting Pos  ", "MIDDLE");
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start ()
    {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop ()
    {
//        telemetry.addData("Opmode", "active");
        //telemetry.update();
        currentBaseAngle = (int) getHeading();  // Degrees
        leftWheelPos = robot.leftDrive.getCurrentPosition();
        rightWheelPos = robot.rightDrive.getCurrentPosition();
        currentBasePos = (leftWheelPos + rightWheelPos)/2.0;
        linearBaseSpeed = currentBasePos - previousBasePos;
        previousBasePos = currentBasePos;
        craneRotatePos = robot.craneRotate.getCurrentPosition();
        craneExtendPos = robot.craneExtend.getCurrentPosition();
        collectorArmPos = robot.armRotate.getCurrentPosition();
        armSpeed =  collectorArmPos - collectorArmPreviousPos;
        collectorArmPreviousPos = collectorArmPos;
        
        switch (stage) {
            case 0: // Initialize
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_COLLECT_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;
                markedTime = runtime.milliseconds();
                stage = 10;
                break;

            case 10: // Deploy jewel flicker arm
                flickerArmTargetPos = Presets.FLICKER_ARM_DEPLOY_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_NEUTRAL_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_COLLECT_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;

                // Allow enough time for the arm to deploy
                if ((runtime.milliseconds() - markedTime) > 600) {
                    stage = 20;
                }
                break;

            case 20: // Scan surroundings for the picture position and jewel color
                flickerArmTargetPos = Presets.FLICKER_ARM_DEPLOY_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_NEUTRAL_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_COLLECT_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;

                picturePos = camera.vuMark;

                 if (getColor() == Presets.COLOR_RED) {
                     ourJewelIsTheFrontOne = !blueAlliance;
                     jewelFound = true;
                 }
                else if (getColor() == Presets.COLOR_BLUE) {
                     ourJewelIsTheFrontOne = blueAlliance;
                     jewelFound = true;
                 }
                 else {
                     jewelFound = false;
                 }

                telemetry.addData("Glyph pos", picturePos);
                //telemetry.addData("Jewel Color", jewelColor());

                markedTime = runtime.milliseconds();
                if (jewelFound){
                    stage = 30;
                }
                else {
                    stage = 50;
                }
                break;

            case 30: // Flick the jewel
                flickerArmTargetPos = Presets.FLICKER_ARM_DEPLOY_POS;
                if (jewelFound){
                    if (ourJewelIsTheFrontOne){
                        flickerFingerTargetPos = Presets.FLICKER_FINGER_FRONT_POS;
                    } else{
                        flickerFingerTargetPos = Presets.FLICKER_FINGER_BACK_POS;
                    }
                }
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_COLLECT_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;

                // Allow just enough time to flick the ball
                if ((runtime.milliseconds() - markedTime) > 500){
                    markedTime = runtime.milliseconds();
                    stage = 40;
                }
                break;

            case 40: // Stow flicker arm before driving
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_NEUTRAL_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;

                if ((runtime.milliseconds() - markedTime) > 500) {
                    markedTime = runtime.milliseconds();
                    if (blueAlliance) {
                        targetBasePos = 24;
                    }
                    else {
                        targetBasePos = -24;
                    }
                    targetBaseAngle = 0;

                    stage = 50;
                }
                break;

            case 50: // Drive off the platform slowly to keep from twisting
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;

                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.3))
                {
                    if (blueAlliance) {
                        if (cornerStartingPos) {
                            targetBasePos = 24;
                            targetBaseAngle = 0;
                        }
                        else {
                            targetBasePos = 24;
                            targetBaseAngle = 90;
                        }
                    }
                    else {
                        if (cornerStartingPos) {
                            targetBasePos = -24;
                            targetBaseAngle = 0;
                        }
                        else {
                            targetBasePos = -24;
                            targetBaseAngle = 90;
                        }
                    }
                    stage = 60;
                }
                break;

            case 60: // Drive to synchronized spot near the cryptobox
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;

                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.6))
                {
                    // Determine the distance to the bonus column on the cryptobox
                    if (image == RelicRecoveryVuMark.LEFT) {
                        if (blueAlliance) {
                            columnOffset = 3;
                        }
                        else {
                            columnOffset = 15;
                        }
                    }
                    else if (image == RelicRecoveryVuMark.RIGHT){
                        if (blueAlliance) {
                            columnOffset = 15;
                        }
                        else {
                            columnOffset = 3;
                        }
                    }
                    else {
                        columnOffset = 9;   // Center position by default
                    }
                    if (blueAlliance) {
                        if (cornerStartingPos) {
                            targetBasePos = 24 + columnOffset;
                            targetBaseAngle = 0;
                        }
                        else {
                            targetBasePos = 24 + columnOffset;
                            targetBaseAngle = 90;
                        }
                    }
                    else {
                        if (cornerStartingPos) {
                            targetBasePos = -24 + columnOffset;
                            targetBaseAngle = 0;
                        }
                        else {
                            targetBasePos = -24 + columnOffset;
                            targetBaseAngle = 90;
                        }
                    }
                    stage = 70;
                }
                break;

            case 70: //Drive in front of the correct column
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;

                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.6))
                {
                    // Set the angle to face the cryptobox
                    if (blueAlliance) {
                        if (cornerStartingPos) {
                            targetBasePos = 24 + columnOffset;
                            targetBaseAngle = -90;
                        }
                        else {
                            targetBasePos = 24 + columnOffset;
                            targetBaseAngle = 0;
                        }
                    }
                    else {
                        if (cornerStartingPos) {
                            targetBasePos = -24 + columnOffset;
                            targetBaseAngle = -90;
                        }
                        else {
                            targetBasePos = -24 + columnOffset;
                            targetBaseAngle = 180;
                        }
                    }
                    stage = 80;
                }
                break;

            case 80: //Turn to face cryptobox
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;

                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.6))
                {
                    if (blueAlliance) {
                        if (cornerStartingPos) {
                            targetBasePos = 24 + columnOffset + 16;
                            targetBaseAngle = -90;
                        }
                        else {
                            targetBasePos = 24 + columnOffset + 16;
                            targetBaseAngle = 0;
                        }
                    }
                    else {
                        if (cornerStartingPos) {
                            targetBasePos = -24 + columnOffset + 16;
                            targetBaseAngle = -90;
                        }
                        else {
                            targetBasePos = -24 + columnOffset + 16;
                            targetBaseAngle = 180;
                        }
                    }
                    // Remember the baseline position for scoring so we can drive from here
                    baseScorePos  = targetBasePos;
                    baseScoreAngle = targetBaseAngle;

                    stage = 90;
                }
                 break;

            case 90: //Drive to Score
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;

                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.6))
                {
                    markedTime = runtime.milliseconds();
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle;
                    stage = 100;
                }
                break;

            case 100: //Release Cube
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_SCORE_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_SCORE_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;

                PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.6);

                if ((runtime.milliseconds() - markedTime) > 200) {
                    markedTime = runtime.milliseconds();
                    targetBasePos = baseScorePos - 8;
                    targetBaseAngle = baseScoreAngle;
                    stage = 110;
                }
                break;

            case 110: //Back up a little to clear the cryptobox
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_SCORE_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_SCORE_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
                
                PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.6);

                break;

        }

        armMotorCmd = PinkPD.getMotorCmd(0.02, 0.01, collectorArmTargetPos - collectorArmPos, armSpeed);
        robot.armRotate.setPower(armMotorCmd);       
               
        robot.collectorRotate.setPosition(collectorRotateTargetPos);
        robot.collectorFinger1.setPosition(collectorFinger1TargetPos);
        robot.collectorFinger2.setPosition(collectorFinger2TargetPos);
        robot.flickerArm.setPosition(flickerArmTargetPos);
        robot.flickerFinger.setPosition(flickerFingerTargetPos);

        robot.craneClaw.setPosition(Presets.CRANE_CLAW_CLOSE_POS);
        robot.craneWrist.setPosition(Presets.CRANE_WRIST_STOW_POS);
        robot.craneRotate.setPower(PinkPD.getMotorCmd(0.01, 0.0, Presets.CRANE_ROTATE_MIN_POS - craneRotatePos, 0.0));
        robot.craneExtend.setPower(PinkPD.getMotorCmd(0.01, 0.0, Presets.CRANE_EXTEND_MIN_POS - craneExtendPos, 0.0));

        telemetry.addData("Stage ", stage);
        telemetry.update();
    }

    public double getHeading () {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
    public int getColor(){

        // Calculate Correlated Color Temperature (CCT)
        double R = robot.colorSensor.red();
        double G = robot.colorSensor.green();
        double B = robot.colorSensor.blue();

        int currentColor = Presets.COLOR_NONE;

        // First check if the distance is less than 6cm
        if(robot.distanceSensor.getDistance(DistanceUnit.CM) < 6.0);
        {
            // Calculate CCT
            // Find out CIE tristimulus values (XYZ)
            double X= ((-0.14282)*(R))+((1.54924)*(G))+((-0.95641)*(B));
            double Y= ((-0.32466)*(R))+ ((1.57837)*(G)) + ((-0.73191)*(B)); //=Illuminance
            double Z= ((-0.68202)*(R)) + ((0.77073)*(G)) + ((0.56332)*(B));

            // Calculate the normalized chromaticity values
            double x = X/(X+Y+Z);
            double y = Y/(X+Y+Z);

//            Compute the CCT value
//           double n=(x-0.3320)/(0.1858-y);
//            double colorCCT1=(449*(n*n*n))+(3525*(n*n))+ (6823.3*n)+5520.33;

            // Consolidated Formula (CCT From RGB)
            double n = (((0.23881)*R)+((0.25499)*G)+((-0.58291)*B))/(((0.11109)*R)+((-0.85406)*G)+((0.52289)*B));
            double colorCCT = (449*(n*n*n))+(3525*(n*n))+(6823.3*n) + 5520.33;

            // now check if the intensity is big enough
            if(colorCCT > 7500.0)
            {
                // Check for Blue
                if((B > 10.0) && (B > (R * 2.0))) // If blue is greater than 10 and at least twice as red
                {
                    currentColor = Presets.COLOR_BLUE;
                } else
                if((R > 10.0) && (R > (B * 2.0))) // If red is greater than 10 and at least twice as blue
                {
                    currentColor = Presets.COLOR_RED;
                }
            } // if intensity of any color is high enough
        } // If sensor distance is close
        return currentColor;
    }

/*
    public boolean driveToPos (double targetPos, double targetAngle, int currentAngle, double leftEnc, double rightEnc,
                               double linearVelocity, double angularVelocity, double maxPower)
    {
        double targetPosCounts = targetPos * COUNTS_PER_INCH;
        telemetry.addData("TargetPositionCounts", targetPosCounts);
        double leftWheelPos = leftEnc;
        telemetry.addData("LeftWheelPos", leftWheelPos);
        double rightWheelPos = rightEnc;
        telemetry.addData("RightWheelPos", rightWheelPos);
        double angleErrorDegrees = targetAngle - currentAngle;
        telemetry.addData("AngleErrorDegrees", angleErrorDegrees);
        double currentPosCounts = (leftWheelPos + rightWheelPos) / 2.0;
        telemetry.addData("CurrentPosCounts", currentPosCounts);
        double angleOffset;
        double linearError = targetPosCounts - currentPosCounts;
        telemetry.addData("LinearError", linearError);
        double angularError = targetAngle - currentAngle;
        telemetry.addData("AngularError", angularError);
        double motorCmd = PinkPD.getMotorCmd(0.1, 0.01, linearError, linearVelocity);

        // Determine the baseline motor speed command
        motorCmd = Range.clip(motorCmd, -0.5, 0.5);

        // Determine and add the angle offset
        angleOffset = PinkPD.getMotorCmd(0.02, 0.02, angularError, angularVelocity);
        telemetry.addData("AngleOffset", angleOffset);
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
        telemetry.addData("MaxPower", maxPower);
        telemetry.addData("RightMotorCommand", rightMotorCmd);
        telemetry.addData("LeftMotorCommand", leftMotorCmd);

        if ((Math.abs(linearError) < POSITION_THRESHOLD) && (Math.abs(angleErrorDegrees) < ANGLE_THRESHOLD))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
*/
    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop ()
    {
    }

}
