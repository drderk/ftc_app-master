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

    static final double COUNTS_PER_INCH = 49.5;  // Base travel
    static final double POSITION_THRESHOLD = 10.0;   // Counts
    static final double ANGLE_THRESHOLD = 5.0;     // Degrees

    static final double FLICKER_ARM_STOW_POSITION = -1;
    static final double FLICKER_ARM_DEPLOY_POSITION = 1;

    static final double FLICKER_FINGER_STOW_POSITION = -1;
    static final double FLICKER_FINGER_NEUTRAL_POSITION = 0;
    static final double FLICKER_FINGER_FRONT_POSITION = 0.2;
    static final double FLICKER_FINGER_BACK_POSITION = -0.2;

    static final double COLLECTOR_ARM_COLLECT_POS = 0;
    static final double COLLECTOR_ARM_LOW_SCORE_POS = 40;
    static final double COLLECTOR_ARM_HIGH_SCORE_POS = 250;

    static final double COLLECTOR_FINGER_GRAB_POSITION = -1;
    static final double COLLECTOR_FINGER_COLLECT_POSITION = 1;
    static final double COLLECTOR_FINGER_SCORE_POSITION = -0.4;
    static final double COLLECTOR_ROTATE_UPRIGHT_POSITION = -1;
    static final double COLLECTOR_ROTATE_INVERTED_POSITION = 1;

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
    boolean blueAlliance;        // Selected alliance color
    boolean cornerStartingPos;   // Corner or middle starting position
    double markedTime;
    boolean ourJewelIsTheFrontOne;

    VuMarks camera = new VuMarks();
    RelicRecoveryVuMark picturePos;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init ()
    {
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
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// INSERT ALLIANCE COLOR SELECTION AND STARTING POSITION HERE
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
        telemetry.addData("Opmode", "active");
        //telemetry.update();
        currentBaseAngle = (int) GetHeading();  // Degrees
        switch (stage)
        {
            case 0: // Initialize
                flickerArmTargetPos = FLICKER_ARM_STOW_POSITION;
                flickerFingerTargetPos = FLICKER_FINGER_STOW_POSITION;
                collectorFinger1TargetPos = COLLECTOR_FINGER_GRAB_POSITION;
                collectorFinger2TargetPos = COLLECTOR_FINGER_GRAB_POSITION;
                collectorRotateTargetPos = COLLECTOR_ROTATE_UPRIGHT_POSITION;
                collectorArmTargetPos = COLLECTOR_ARM_COLLECT_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;

                stage = 30;
                break;

            case 10: // Scan surroundings
                flickerArmTargetPos = FLICKER_ARM_STOW_POSITION;
                flickerFingerTargetPos = FLICKER_FINGER_STOW_POSITION;
                collectorFinger1TargetPos = COLLECTOR_FINGER_GRAB_POSITION;
                collectorFinger2TargetPos = COLLECTOR_FINGER_GRAB_POSITION;
                collectorRotateTargetPos = COLLECTOR_ROTATE_UPRIGHT_POSITION;
                collectorArmTargetPos = COLLECTOR_ARM_COLLECT_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;

                picturePos = camera.vuMark;
// ADD LOGIC TO SCAN THE COLOR SENSOR
                ourJewelIsTheFrontOne = true;
                telemetry.addData("Glyph pos", picturePos);
                //telemetry.addData("Jewel Color", jewelColor());

                markedTime = runtime.milliseconds();

                stage = 20;
                break;

            case 20: //lower the jewel arm
                flickerArmTargetPos = FLICKER_ARM_DEPLOY_POSITION;
                flickerFingerTargetPos = FLICKER_FINGER_NEUTRAL_POSITION;
                collectorFinger1TargetPos = COLLECTOR_FINGER_GRAB_POSITION;
                collectorFinger2TargetPos = COLLECTOR_FINGER_GRAB_POSITION;
                collectorRotateTargetPos = COLLECTOR_ROTATE_UPRIGHT_POSITION;
                collectorArmTargetPos = COLLECTOR_ARM_COLLECT_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;

                telemetry.addData("Stage", stage);
// ADD SOME TIME DELAY HERE TO ALLOW THE ARM TO DEPLOY ` 0.5 seconds
                if ((runtime.milliseconds() - markedTime) > 500){
                    markedTime = runtime.milliseconds();
                    stage = 30;
                }
                break;

            case 30: // Flick the jewel
                flickerArmTargetPos = FLICKER_ARM_DEPLOY_POSITION;
                if (ourJewelIsTheFrontOne){
                    flickerFingerTargetPos = FLICKER_FINGER_FRONT_POSITION;
                } else{
                    flickerFingerTargetPos = FLICKER_FINGER_BACK_POSITION;
                }
                collectorFinger1TargetPos = COLLECTOR_FINGER_GRAB_POSITION;
                collectorFinger2TargetPos = COLLECTOR_FINGER_GRAB_POSITION;
                collectorRotateTargetPos = COLLECTOR_ROTATE_UPRIGHT_POSITION;
                collectorArmTargetPos = COLLECTOR_ARM_COLLECT_POS;
                targetBasePos = 0;
                targetBaseAngle = 0;

                telemetry.addData("Stage", stage);
                //drive completely off ramp
                break;

            case 40: //turn
                collectPos = 0;
                liftPos = 0;
                jewelPos = 0;
                grabPos = 0;
                rotatePos = 0;
                extendPos = 0;
                targetPos = 45;
                targetAngle = 90;

                telemetry.addData("Stage", stage);

                if (driveToPos(targetPos, targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                {
                    stage = 100;
                }
                else
                {
                    stage = 40;
                }
                break;

            case 50: //drive to center
                collectPos = 0;
                liftPos = 0;
                jewelPos = 0;
                grabPos = 0;
                rotatePos = 0;
                extendPos = 0;
                targetPos = 90;
                targetAngle = 90;

                telemetry.addData("Stage", stage);
                if (driveToPos(targetPos, targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                {
                    stage = 60;
                }
                else
                {
                    stage = 40;
                }
                break;
            case 60: //drive to center
                collectPos = 0;
                liftPos = 0;
                jewelPos = 0;
                grabPos = 0;
                rotatePos = 0;
                extendPos = 0;
                targetPos = 90;
                targetAngle = 90;
                telemetry.addData("Stage", stage);

                if (driveToPos(targetPos, targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                {
                    stage = 70;
                }
                else
                {
                    stage = 40;
                }
                break;
            case 70: //drive to center
                collectPos = 0;
                liftPos = 0;
                jewelPos = 0;
                grabPos = 0;
                rotatePos = 0;
                extendPos = 0;
                targetPos = 90;
                targetAngle = 90;
                telemetry.addData("Stage", stage);

                if (driveToPos(targetPos, targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                {
                    stage = 70;
                }
                else
                {
                    stage = 40;
                }
                break;

            case 100: //end
                collectPos = 0;
                liftPos = 0;
                jewelPos = 0;
                grabPos = 0;
                rotatePos = 0;
                extendPos = 0;
                driveToPos(targetPos, targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1);
                telemetry.addData("Stage", stage);
                break;
        }

        //set all values
               /* robot.collect.setPosition(collectPos);
                robot.lift.setPower(liftPos);
                robot.jewel.setPosition(jewelPos);
                robot.grab.setPosition(grabPos);
                robot.rotate.setPosition(rotatePos);
                robot.extend.setPower(extendPos);*/
        telemetry.update();
    }

    public double GetHeading ()
    {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop ()
    {
    }

}