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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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

@Autonomous (name = "Auto")
//@Disabled
public class Auto extends OpMode
{

    /* Declare OpMode members. */
    private Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware
    private BNO055IMU imu;
    private double currentBaseAngle;
    private int stage = 0;
    private double leftWheelMotorCmd, rightWheelMotorCmd, armMotorCmd;
    private double collectorArmTargetPos;
    private double flickerArmTargetPos, flickerFingerTargetPos;
    private double collectorFinger1TargetPos, collectorFinger2TargetPos, collectorRotateTargetPos;
    private double targetBasePos, targetBaseAngle;
    private double baseScorePos, baseScoreAngle;
    private boolean jewelFound = false;
    private boolean blueAlliance;        // Selected alliance color
    private boolean cornerStartingPos;   // Corner or middle starting position
    private double markedTime; // Represents a set point in time
    private boolean ourJewelIsTheFrontOne;
    private double leftWheelPos = 0, rightWheelPos = 0;
    private double currentBasePos, previousBasePos;
    private double linearBaseSpeed = 0;
    private double columnOffset = 0;
    //private double craneRotatePos = 0;
    //private double craneExtendPos = 0;
    private double collectorArmPos = 0, collectorArmPreviousPos = 0, armSpeed = 0;
    //private double angularSpeed = 0;

    private RelicRecoveryVuMark image = null;
    int jewelColor = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;

    @Override
    public void init ()
    {
        boolean robotAutoConfigured = false;
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters cameraParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        cameraParameters.vuforiaLicenseKey = "AaId2D7/////AAAAGeEOwvh4YkCKim1fP/VA8hpZk/olkYH12xynqz4wP+p4EjzCP4otFEoCxD0cztAeqHF3sVP3DJkAIOKqVX8UM6YbWpaaZPA3fK1YUfNg1Eh7A47eCRH0zO4hSJZ6fJEnw/NtT+dyv162iRX46R3xsyfB4CZdrHH2Yuxxoa9iWfaLfMdT7p7AWxUjHyujL28oC9xNcv2hJ0QDVbq3om6OzNEbAfkVbUf2q+z/VoWoH6036CL5fzB/ddo2E3Lgiv3PMoGtQyoWDtAuV6s53CAs/GuSGdv/WmltQtuxcu4w6QrdZIF2SCQ3idYKEPUuv16ranl1/Ayz5OgnYQf4HLRYLgnCRFKXEd7WZPVaLIwM9bJq"; //"ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";


        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        cameraParameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(cameraParameters);

        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
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
        relicTrackables.activate();
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop ()
    {
        if (gamepad1.x)
        {
            blueAlliance = true;
        }
        else if (gamepad1.b)
        {
            blueAlliance = false;
        }
        if (gamepad1.y)
        {
            cornerStartingPos = true;
        }
        else if (gamepad1.a)
        {
            cornerStartingPos = false;
        }
        if (blueAlliance)
        {
            telemetry.addData("Alliance Color", "BLUE");
        }
        else
        {
            telemetry.addData("Alliance Color", "RED");
        }
        if (cornerStartingPos)
        {
            telemetry.addData("Starting Pos  ", "CORNER");
        }
        else
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
//        currentBaseAngle = getHeading();  // Degrees
        leftWheelPos = robot.leftDrive.getCurrentPosition();
        rightWheelPos = robot.rightDrive.getCurrentPosition();
        currentBasePos = (leftWheelPos + rightWheelPos) / 2.0;
        linearBaseSpeed = currentBasePos - previousBasePos;
        previousBasePos = currentBasePos;
//        craneRotatePos = robot.craneRotate.getCurrentPosition();
//        craneExtendPos = robot.craneExtend.getCurrentPosition();
        collectorArmPos = robot.armRotate.getCurrentPosition();
        armSpeed = collectorArmPos - collectorArmPreviousPos;
        collectorArmPreviousPos = collectorArmPos;

        switch (stage)
        {
            case 0: // Initialize
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = 0;
                targetBasePos = 0;
                targetBaseAngle = 0;
                // Set the claw here and leave it since we don't use it
                robot.craneClaw.setPosition(Presets.CRANE_CLAW_CLOSE_POS);
                robot.craneWrist.setPosition(Presets.CRANE_WRIST_LATCH_POS);
                PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.3);

                markedTime = runtime.milliseconds();
                stage = 10;
                break;

            case 10: // Deploy jewel flicker arm
                flickerArmTargetPos = Presets.FLICKER_ARM_DEPLOY_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_NEUTRAL_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = 0;
                targetBasePos = 0;
                targetBaseAngle = 0;

                // Allow enough time for the arm to deploy
                if ((runtime.milliseconds() - markedTime) > 1000)
                {
                    markedTime = runtime.milliseconds();
                    stage = 20;
                }
                break;

            case 20: // Scan surroundings for the picture position and jewel color
                flickerArmTargetPos = Presets.FLICKER_ARM_DEPLOY_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_NEUTRAL_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = 0;
                targetBasePos = 0;
                targetBaseAngle = 0;

                // Scan for the image and jewel color
                image = getImage();
                jewelColor = getColor();
                if (jewelColor == Presets.COLOR_RED)
                {
                    ourJewelIsTheFrontOne = !blueAlliance;
                    jewelFound = true;
                }
                else if (jewelColor == Presets.COLOR_BLUE)
                {
                    ourJewelIsTheFrontOne = blueAlliance;
                    jewelFound = true;
                }
                else
                {
                    jewelFound = false;
                }

                if (jewelFound && ((runtime.milliseconds() - markedTime) > 1500))
                {
                    markedTime = runtime.milliseconds();
                    stage = 30;
                }
                else if ((runtime.milliseconds() - markedTime) > 2500)
                {
                    markedTime = runtime.milliseconds();
                    stage = 50;
                }
                break;

            case 30: // Flick the jewel
                flickerArmTargetPos = Presets.FLICKER_ARM_DEPLOY_POS;
                if (jewelFound)
                {
                    if (ourJewelIsTheFrontOne)
                    {
                        flickerFingerTargetPos = Presets.FLICKER_FINGER_FRONT_POS;
                    }
                    else
                    {
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
                if ((runtime.milliseconds() - markedTime) > 2000)
                {
                    markedTime = runtime.milliseconds();
                    stage = 40;
                }
                break;

            case 40: // Stow flicker arm before driving
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_TRAVEL_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;

                if ((runtime.milliseconds() - markedTime) > 500)
                {
                    markedTime = runtime.milliseconds();
                    stage = 50;
                }
                break;

            case 50: // Drive off the platform slowly to keep from twisting
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_TRAVEL_POS;

                if (blueAlliance)
                {
                    targetBasePos = 25;
                }
                else
                {
                    targetBasePos = -25;
                }
                targetBaseAngle = 0;

                currentBaseAngle = getHeading();  // Degrees
                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.3))
                {
                    markedTime = runtime.milliseconds();
                    stage = 60;
                }
                break;

            case 60: // Drive to synchronized spot near the cryptobox
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_TRAVEL_POS;
                if (blueAlliance)
                {
                    if (cornerStartingPos)
                    {
                        targetBasePos = 25;
                        targetBaseAngle = 0;
                    }
                    else
                    {
                        targetBasePos = 25;
                        targetBaseAngle = -90;
                    }
                }
                else
                {
                    if (cornerStartingPos)
                    {
                        targetBasePos = -25;
                        targetBaseAngle = 0;
                    }
                    else
                    {
                        targetBasePos = -25;
                        targetBaseAngle = 90;
                    }
                }

                currentBaseAngle = getHeading();  // Degrees
                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.3) && ((runtime.milliseconds() - markedTime) > 3000))
                {
                    stage = 70;
                }
                break;

            case 70: //Drive in front of the correct column
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_TRAVEL_POS;
                // Determine the distance to the bonus column on the cryptobox
                if (image == RelicRecoveryVuMark.LEFT)
                {
                    if (blueAlliance)
                    {
                        columnOffset = 4.5;
                    }
                    else
                    {
                        columnOffset = 18.5;
                    }
                }
                else if (image == RelicRecoveryVuMark.RIGHT)
                {
                    if (blueAlliance)
                    {
                        columnOffset = 18.5;
                    }
                    else
                    {
                        columnOffset = 3.5;
                    }
                }
                else
                {
                    columnOffset = 12;   // Center position by default
                }
                if (blueAlliance)
                {
                    if (cornerStartingPos)
                    {
                        targetBasePos = 25 + columnOffset;
                        targetBaseAngle = 0;
                    }
                    else
                    {
                        targetBasePos = 24 + columnOffset + 2.5;
                        targetBaseAngle = -90;
                    }
                }
                else
                {
                    if (cornerStartingPos)
                    {
                        targetBasePos = -25 - columnOffset;
                        targetBaseAngle = 0;
                    }
                    else
                    {
                        targetBasePos = -25 - columnOffset;
                        targetBaseAngle = 90;
                    }
                }
                currentBaseAngle = getHeading();  // Degrees
                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.3))
                {
                    stage = 80;
                    markedTime = runtime.milliseconds();
                }
                break;

            case 80: //Turn to face cryptobox
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_TRAVEL_POS;
                // Set the angle to face the cryptobox
                if (blueAlliance)
                {
                    if (cornerStartingPos)
                    {
                        targetBasePos = 25 + columnOffset;
                        targetBaseAngle = -90;
                    }
                    else
                    {
                        targetBasePos = 24 + columnOffset + 1.5;
                        targetBaseAngle = 0;
                    }
                }
                else
                {
                    if (cornerStartingPos)
                    {
                        targetBasePos = -25 - columnOffset;
                        targetBaseAngle = 90;
                    }
                    else
                    {
                        targetBasePos = -25 - columnOffset;
                        targetBaseAngle = 180;
                    }
                }
                currentBaseAngle = getHeading();  // Degrees
                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.3) && ((runtime.milliseconds() - markedTime) > 3000))
                {
                    stage = 90;
                }
                break;

            case 90: //Drive forward to Score
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_GRAB_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
                if (blueAlliance)
                {
                    if (cornerStartingPos)
                    {
                        targetBasePos = 25 + columnOffset + 10;
                        targetBaseAngle = -90;
                    }
                    else
                    {
                        targetBasePos = 24 + columnOffset + 8 + 1.5;
                        targetBaseAngle = 0;
                    }
                }
                else
                {
                    if (cornerStartingPos)
                    {
                        targetBasePos = -25 - columnOffset + 10;
                        targetBaseAngle = 90;
                    }
                    else
                    {
                        targetBasePos = -25 - columnOffset + 10;
                        targetBaseAngle = 180;
                    }
                }
                // Remember the baseline position for scoring so we can drive from here
                baseScorePos = targetBasePos;
                baseScoreAngle = targetBaseAngle;

                currentBaseAngle = getHeading();  // Degrees
                if (PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.25))
                {
                    markedTime = runtime.milliseconds();
                    stage = 100;
                }
                break;

            case 100: //Release Cube
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_COLLECT_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_COLLECT_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
                targetBasePos = baseScorePos;
                targetBaseAngle = baseScoreAngle;

                currentBaseAngle = getHeading();  // Degrees
                PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.3);

                if ((runtime.milliseconds() - markedTime) > 200)
                {
                    markedTime = runtime.milliseconds();
                    stage = 110;
                }
                break;

            case 110: //Back up a little to clear the cryptobox
                flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
                flickerFingerTargetPos = Presets.FLICKER_FINGER_STOW_POS;
                collectorFinger1TargetPos = Presets.COLLECTOR_FINGER_COLLECT_POS;
                collectorFinger2TargetPos = Presets.COLLECTOR_FINGER_COLLECT_POS;
                collectorRotateTargetPos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
                collectorArmTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
                targetBasePos = baseScorePos - 8;
                targetBaseAngle = baseScoreAngle;

                currentBaseAngle = getHeading();  // Degrees
                PinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.3);

                break;

        }

        armMotorCmd = PinkPD.getMotorCmd(0.02, 0.01, collectorArmTargetPos - collectorArmPos, armSpeed);
        robot.armRotate.setPower(armMotorCmd);

        robot.collectorRotate.setPosition(collectorRotateTargetPos);
        robot.collectorFinger1.setPosition(collectorFinger1TargetPos);
        robot.collectorFinger2.setPosition(collectorFinger2TargetPos);
        robot.flickerArm.setPosition(flickerArmTargetPos);
        robot.flickerFinger.setPosition(flickerFingerTargetPos);

//        robot.craneClaw.setPosition(Presets.CRANE_CLAW_CLOSE_POS);
//        robot.craneWrist.setPosition(Presets.CRANE_WRIST_LATCH_POS);
//        robot.craneRotate.setPower(-0.1);
//        robot.craneRotate.setPower(PinkPD.getMotorCmd(0.01, 0.0, Presets.CRANE_ROTATE_MIN_POS - craneRotatePos, 0.0));
//        robot.craneExtend.setPower(-0.1);
//        robot.craneExtend.setPower(PinkPD.getMotorCmd(0.01, 0.0, Presets.CRANE_EXTEND_MIN_POS - craneExtendPos, 0.0));

        robot.leftDrive.setPower(PinkNavigate.getLeftMotorCmd());
        robot.rightDrive.setPower(PinkNavigate.getRightMotorCmd());

        telemetry.addData("Stage  ", stage);
        telemetry.addData("Image  ", image);
        telemetry.addData("Color  ", jewelColor);

//        telemetry.addData("Base Angle  ",  currentBaseAngle);

    }

    private double getHeading ()
    {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    private RelicRecoveryVuMark getImage ()
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null)
            {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        }
        return vuMark;
    }

    private String format (OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public int getColor ()
    {

        // Calculate Correlated Color Temperature (CCT)
        double R = robot.colorSensor.red();
        double G = robot.colorSensor.green();
        double B = robot.colorSensor.blue();

        int currentColor = Presets.COLOR_NONE;

        // First check if the distance is less than 6cm
        if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 6.0)
        {
            // Calculate CCT
            // Find out CIE tristimulus values (XYZ)
            double X = ((-0.14282) * (R)) + ((1.54924) * (G)) + ((-0.95641) * (B));
            double Y = ((-0.32466) * (R)) + ((1.57837) * (G)) + ((-0.73191) * (B)); //=Illuminance
            double Z = ((-0.68202) * (R)) + ((0.77073) * (G)) + ((0.56332) * (B));

            // Calculate the normalized chromaticity values
            double x = X / (X + Y + Z);
            double y = Y / (X + Y + Z);

//            Compute the CCT value
//           double n=(x-0.3320)/(0.1858-y);
//            double colorCCT1=(449*(n*n*n))+(3525*(n*n))+ (6823.3*n)+5520.33;

            // Consolidated Formula (CCT From RGB)
            double n = (((0.23881) * R) + ((0.25499) * G) + ((-0.58291) * B)) / (((0.11109) * R) + ((-0.85406) * G) + ((0.52289) * B));
            double colorCCT = (449 * (n * n * n)) + (3525 * (n * n)) + (6823.3 * n) + 5520.33;

            // Now check if the intensity is big enough
            if (colorCCT > 7500.0)
            {
                // Check for Blue
                if ((B > 10.0) && (B > (R * 1.5))) // If blue is greater than 10 and at least twice as red
                {
                    currentColor = Presets.COLOR_BLUE;
                }
                else if ((R > 10.0) && (R > (B * 1.5))) // If red is greater than 10 and at least twice as blue
                {
                    currentColor = Presets.COLOR_RED;
                }
            } // if intensity of any color is high enough
        } // If sensor distance is close
        return currentColor;
    }

    /*
    * Code to run ONCE after the driver hits STOP
    */
    @Override
    public void stop ()
    {
    }

}
