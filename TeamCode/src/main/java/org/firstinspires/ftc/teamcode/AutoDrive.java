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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive", group="Pushbot")
public class AutoDrive extends LinearOpMode {

    //declare variables
    Hardware         robot   = new Hardware();   // Use a Pushbot's hardware
    PinkNavigate pinkNavigate = new PinkNavigate();
    private ElapsedTime     runtime = new ElapsedTime();
    BNO055IMU imu;
    int currentAngle;
    int stage = 0;
    boolean auto = true;
    boolean topBal;
    
    //motor and servo setting values
    double collectPos = 0;
    double liftPos = 0;
    double jewelPos = 0;
    double grabPos = 0;
    double rotatePos = 0;
    double extendPos= 0;
    double targetPos = 0;
    double targetAngle = 0;

    VuMarks camera = new VuMarks();
    RelicRecoveryVuMark picturePos;

    @Override
    public void runOpMode() {
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
        robot.init(hardwareMap);
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        PinkNavigate.robot = robot;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Waiting for start");    //
        telemetry.update();
        waitForStart();

        //Motion Start
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                currentAngle = (int) GetHeading();
                switch (stage) {
                    case 0: //initialize
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        stage = 3;
                        targetPos = 0;
                        targetAngle = 0;
                        break;

                    case 1: //scan surroundings
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        targetPos = 0;
                        targetAngle = 0;
                        
                        picturePos = camera.vuMark;
                        telemetry.addData("Glyph pos", picturePos);
                        //telemetry.addData("Jewel Color", jewelColor());
                        sleep(1000);
                        stage = 2;
                        break;

                    case 2: //lower the jewel arm
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 1;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        targetPos = 0;
                        targetAngle = 0;
                        
                    	sleep(1000);
                        telemetry.addData("Stage", stage);
                        stage = 3;
                        break;

                    case 3: //lift up jewel mechanism and drive forward
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        targetPos = 45;
                        targetAngle = 0;
                        
                        //drive completely off ramp
                        if (PinkNavigate.driveToPos(targetPos, targetAngle, currentAngle, 0, 0, 1)) {
                        	stage = 100;
                        }
                        else {
                        	stage = 3;
                        }
                        break;
                        
                    case 4: //turn
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        targetPos = 45;
                        targetAngle = 90;
                        
                        if (PinkNavigate.driveToPos(targetPos, targetAngle, currentAngle, 0, 0, 1)) {
                        	stage = 100;
                        }
                        else {
                        	stage = 4;
                        }
                        break;
                        
                    case 5: //drive to center
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        targetPos = 90;
                        targetAngle = 90;
                        
                        if (PinkNavigate.driveToPos(targetPos, targetAngle, currentAngle, 0, 0, 1)) {
                        	stage = 6;
                        }
                        else {
                        	stage = 4;
                        }
                        break;
                    case 6: //drive to center
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        targetPos = 90;
                        targetAngle = 90;
                        
                        if (PinkNavigate.driveToPos(targetPos, targetAngle, currentAngle, 0, 0, 1)) {
                        	stage = 7;
                        }
                        else {
                        	stage = 4;
                        }
                        break;
                    case 7: //drive to center
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        targetPos = 90;
                        targetAngle = 90;
                        
                        if (PinkNavigate.driveToPos(targetPos, targetAngle, currentAngle, 0, 0, 1)) {
                        	stage = 7;
                        }
                        else {
                        	stage = 4;
                        }
                        break;
                        
                    case 100: //end
                    	collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos= 0;
                        targetPos = 0;
                        targetAngle = 0;
                        PinkNavigate.driveToPos(targetPos, targetAngle, currentAngle, 0, 0, 0);
                        auto = false;
                        break;

                }
                
                //set all values
               /* robot.collect.setPosition(collectPos);
                robot.lift.setPower(liftPos);
                robot.jewel.setPosition(jewelPos);
                robot.grab.setPosition(grabPos);
                robot.rotate.setPosition(rotatePos);
                robot.extend.setPower(extendPos);*/
                
            }
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
   */

    public double GetHeading(){
        Orientation angles;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
    /*public  String jewelColor() {
        if (robot.colorSensor.red() > 3) {
            return "Red";
        }
        else{
            return "Blue";
        }*/
    }
