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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * This file provides Teleop driving for a single robot using the Hardware class.
 * The code is structured as an Iterative OpMode.
*/

@TeleOp (name = "PINK Teleop", group = "Pushbot")
public class Teleop extends OpMode
{

    /* Declare OpMode members. */
    private Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware
    private BNO055IMU imu;
    private boolean collectorRotateButtonWasntAlreadyPressed;
    private boolean craneClawOpenButtonWasntAlreadyPressed;
    private double previousArmPos = 0;
    private double previousCraneRotatePos = 0;
    private double armSpeed = 0; // Max
    private double craneRotateSpeed = 0;
    private double leftJoystick, rightJoystick;
    private double leftWheelsMotorCmd, rightWheelsMotorCmd, armMotorCmd;
    private double craneRotateMotorCmd, craneExtendMotorCmd;
    private double collectorFinger1Pos = Presets.COLLECTOR_HOLD;
    private double collectorFinger2Pos = Presets.COLLECTOR_HOLD;
    private double collectorFinger3Pos = Presets.COLLECTOR_HOLD;
    private double collectorFinger4Pos = Presets.COLLECTOR_HOLD;
    private double collectorRotatePos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
    private double armTargetPos = 0;
    private double armCurrentPos = 0;
    private double craneRotateTargetPos = 0;
    private double craneRotateCurrentPos = 0;
    private double craneExtendTargetPos = 0;
    private double craneExtendCurrentPos = 0;
    private double craneWristTargetPos = Presets.CRANE_WRIST_STOW_POS;
    private double craneClawTargetPos = Presets.CRANE_CLAW_CLOSE_POS;
    private double flickerFingerTargetPos = Presets.FLICKER_FINGER_NEUTRAL_POS;
    private double flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
    private boolean telemetryActivated = false;
    private boolean baseHoldActivated  = false;
    private double  targetBase         = 0;
    private double  targetAngle        = 0;
    private double  currentAngle      = 0;
    private double  currentBase        = 0;
    private double  armMax              =0.15;
    private double  armAngle            = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init ()
    {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
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

        robot.flickerArm.setPosition(Presets.FLICKER_ARM_STOW_POS);
        robot.flickerFinger.setPosition(Presets.FLICKER_FINGER_STOW_POS);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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
        // Do up-front calculations for control loops
        armCurrentPos = robot.armRotate.getCurrentPosition();
        armSpeed = armCurrentPos - previousArmPos;
        previousArmPos = armCurrentPos;
        armAngle = armCurrentPos * 0.12;
        armAngle = armAngle * (Math.PI / 180.0);


        craneExtendCurrentPos = robot.craneExtend.getCurrentPosition();

        craneRotateCurrentPos = robot.craneRotate.getCurrentPosition();
        craneRotateSpeed = craneRotateCurrentPos - previousCraneRotatePos;
        previousCraneRotatePos = craneRotateCurrentPos;

        // Close the fingers if no other commands are given
        collectorFinger1Pos = Presets.COLLECTOR_HOLD;
        collectorFinger2Pos = Presets.COLLECTOR_HOLD;
        collectorFinger3Pos = Presets.COLLECTOR_HOLD;
        collectorFinger4Pos = Presets.COLLECTOR_HOLD;

        // BASE CONTROL /////////////////////////////////////////////////////
        // Run wheels in tank mode (The joystick is negative when pushed forward, so negate it)
        leftJoystick = -gamepad1.left_stick_y;
        rightJoystick = -gamepad1.right_stick_y;

        currentAngle = getHeading();
        currentBase   = (robot.leftDrive.getCurrentPosition() + robot.rightDrive.getCurrentPosition()) / 2;


        if (gamepad1.left_trigger > 0.2)
        {
            leftWheelsMotorCmd = leftJoystick * 1.0;
            rightWheelsMotorCmd = rightJoystick * 1.0;
        }
        else
        {
            leftWheelsMotorCmd = leftJoystick * 0.6;
            rightWheelsMotorCmd = rightJoystick * 0.6;
        }

        if (gamepad1.a) {
            leftWheelsMotorCmd = -.3;
            rightWheelsMotorCmd = -.3;
        }

        // COLLECTOR CONTROL /////////////////////////////////////////////////////
        // Scoring finger position
        if (gamepad2.b)
        {
            collectorFinger1Pos = Presets.COLLECTOR_EJECT;     // Slightly open to release glyph
            collectorFinger2Pos = Presets.COLLECTOR_EJECT;     // Slightly open to release glyph
            collectorFinger3Pos = Presets.COLLECTOR_EJECT;     // Slightly open to release glyph
            collectorFinger4Pos = Presets.COLLECTOR_EJECT;     // Slightly open to release glyph
        }

        // Open bottom fingers
        if (gamepad1.right_bumper)
        {
            if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS)
            {
                collectorFinger1Pos = Presets.COLLECTOR_COLLECT;
                collectorFinger2Pos = Presets.COLLECTOR_COLLECT;
            }
            else
            {
                collectorFinger3Pos = Presets.COLLECTOR_COLLECT;
                collectorFinger4Pos = Presets.COLLECTOR_COLLECT;
            }
        }

        // Open top fingers
        if (gamepad1.left_bumper)
        {
            if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS)
            {
                collectorFinger3Pos = Presets.COLLECTOR_COLLECT;
                collectorFinger4Pos = Presets.COLLECTOR_COLLECT;
            }
            else
            {
                collectorFinger1Pos = Presets.COLLECTOR_COLLECT;
                collectorFinger2Pos = Presets.COLLECTOR_COLLECT;
            }
        }

        // Toggle the collector rotate position
        if ((gamepad2.right_bumper) && (collectorRotateButtonWasntAlreadyPressed))
        {
            if (armCurrentPos < Presets.COLLECTOR_ARM_TRAVEL_POS){
                armTargetPos = Presets.COLLECTOR_ARM_TRAVEL_POS;
            }
            collectorRotateButtonWasntAlreadyPressed = false;
            if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS)
            {
                collectorRotatePos = Presets.COLLECTOR_ROTATE_INVERTED_POS;
            }
            else
            {
                collectorRotatePos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
            }
        }

        if (!gamepad2.right_bumper)
        {
            collectorRotateButtonWasntAlreadyPressed = true;
        }

      /*  //max power adjust
        if(gamepad1.dpad_up){
            armMax += 0.01;
        }
        else if (gamepad1.dpad_down){
            armMax -= 0.01;
        }*/
      armMax = 0.15;
//              (0.35 * (Math.sin(armAngle)));

        // ARM CONTROL /////////////////////////////////////////////////////
        // Raise the collector slightly to keep it off the floor
        // or high to score
        if (gamepad2.a)
        {
            armTargetPos = Presets.COLLECTOR_ARM_COLLECT_POS;
        }
        else if (gamepad2.y)
        {
            armTargetPos = Presets.COLLECTOR_ARM_HIGH_SCORE_POS;
        }
        else if (gamepad2.x)
        {
            armTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
        } else if (armTargetPos == Presets.COLLECTOR_ARM_COLLECT_POS){
            armTargetPos = Presets.COLLECTOR_ARM_TRAVEL_POS;
        }
        // Manual arm movement
        if ((gamepad2.right_stick_y > 0.1) || (gamepad2.right_stick_y < -0.1))
        {
            armTargetPos = armTargetPos - (5.0 * gamepad2.right_stick_y);
        }

        if (gamepad2.start) {
            robot.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // CLAW CONTROL /////////////////////////////////////////////////////
        // Toggle the claw finger open/close
        if ((gamepad2.right_trigger > 0.5) && (craneClawOpenButtonWasntAlreadyPressed))
        {
            craneClawOpenButtonWasntAlreadyPressed = false;
            if (craneClawTargetPos == Presets.CRANE_CLAW_OPEN_POS)
            {
                craneClawTargetPos = Presets.CRANE_CLAW_CLOSE_POS;
            }
            else
            {
                craneClawTargetPos = Presets.CRANE_CLAW_OPEN_POS;
            }
        }

        if (gamepad2.right_trigger < 0.5)
        {
            craneClawOpenButtonWasntAlreadyPressed = true;
        }

        // Manual crane wrist control
        if ((gamepad2.left_stick_y > 0.1) || (gamepad2.left_stick_y < -0.1))
        {
            craneWristTargetPos = craneWristTargetPos - (0.02 * gamepad2.left_stick_y);
        }

//        if (gamepad2.left_trigger > 0.5)
//        {
//            craneWristTargetPos = Presets.CRANE_WRIST_SCORE_POS;
//        }

        // CRANE CONTROL /////////////////////////////////////////////////////
        // Crane rotation control
        if (gamepad2.dpad_up)
        {
            craneRotateTargetPos = craneRotateTargetPos + 4;
        }
        else if (gamepad2.dpad_down)
        {
            craneRotateTargetPos = craneRotateTargetPos - 2;
        }

        // Move the crane out
        if (gamepad2.dpad_right)
        {
            if (robot.craneExtend.getCurrentPosition() < Presets.CRANE_EXTEND_MAX_POS)
            {
                craneExtendMotorCmd = 1.0;
            }
            else
            {
                craneExtendMotorCmd = 0;
            }
        }
        // Move the crane in
        else if (gamepad2.dpad_left)
        {
            craneExtendMotorCmd = -0.5;
        }
        else
        {
            craneExtendMotorCmd = 0;
        }

        if (gamepad2.left_bumper)
        {
            craneWristTargetPos = Presets.CRANE_WRIST_SCORE_POS;
        }

        if (gamepad2.left_trigger > 0.5)
        {
            craneWristTargetPos = Presets.CRANE_WRIST_COLLECT_POS;
            craneRotateTargetPos = Presets.CRANE_ROTATE_COLLECT_POS;
            craneClawTargetPos = Presets.CRANE_CLAW_OPEN_POS;
        }

        // Limit position and power
        if (armTargetPos <= Presets.COLLECTOR_ARM_LOW_SCORE_POS)
        {
            armMotorCmd = PinkPD.getMotorCmd(0.004, 0.002, armTargetPos - armCurrentPos, armSpeed);
            armMotorCmd = Range.clip(armMotorCmd, -0.05, 1);
            armMotorCmd *= .6;
        }
        else
        {
            armMotorCmd = PinkPD.getMotorCmd(0.004, 0.002, armTargetPos - armCurrentPos, armSpeed);
            armMotorCmd = Range.clip(armMotorCmd, -0.05, 1);
            armMotorCmd *= .6;
        }

        craneRotateTargetPos = Range.clip(craneRotateTargetPos, Presets.CRANE_ROTATE_MIN_POS, Presets.CRANE_ROTATE_MAX_POS);
        craneExtendTargetPos = Range.clip(craneExtendTargetPos, Presets.CRANE_EXTEND_MIN_POS, Presets.CRANE_EXTEND_MAX_POS);
        craneRotateMotorCmd = PinkPD.getMotorCmd(0.03, 0.02, craneRotateTargetPos - craneRotateCurrentPos, craneRotateSpeed);
        //       craneExtendMotorCmd = PinkPD.getMotorCmd(0.02, 0, craneExtendTargetPos - craneExtendCurrentPos, 0);f
        //       craneExtendMotorCmd = Range.clip(craneExtendMotorCmd, -0.4, 0.4);
        craneWristTargetPos = Range.clip(craneWristTargetPos, Presets.CRANE_WRIST_MIN_POS, Presets.CRANE_WRIST_MAX_POS);

/*
        // Manual flicker control for testing only
        if (gamepad1.y)
        {
            flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
        }
        else if (gamepad1.a)
        {
            flickerArmTargetPos = Presets.FLICKER_ARM_DEPLOY_POS;
        }

        if (gamepad1.b)
        {
            //flickergergerTargetPos += 0.02;
            flickerFingerTargetPos = Presets.FLICKER_FINGER_BACK_POS;
        }
        else if (gamepad1.x)
        {
            //flickerFingerTargetPos -= 0.02;
            flickerFingerTargetPos = Presets.FLICKER_FINGER_FRONT_POS;
        }
        else
        {
           flickerFingerTargetPos = Presets.FLICKER_FINGER_NEUTRAL_POS;
        }
        flickerFingerTargetPos = Range.clip(flickerFingerTargetPos, -1.0, 1.0);
*/
        if (gamepad1.start){
            telemetryActivated = true;
        }
        else {
            telemetryActivated = false;
        }

        // Set powers and positions
        if(baseHoldActivated) {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
        } else {
            robot.leftDrive.setPower(leftWheelsMotorCmd);
            robot.rightDrive.setPower(rightWheelsMotorCmd);
        }
        robot.armRotate.setPower(armMotorCmd);
        robot.collectorFinger1.setPosition(collectorFinger1Pos);
        robot.collectorFinger2.setPosition(collectorFinger2Pos);
        robot.collectorFinger3.setPosition(collectorFinger3Pos);
        robot.collectorFinger4.setPosition(collectorFinger4Pos);
        robot.collectorRotate.setPosition(collectorRotatePos);
        robot.craneRotate.setPower(craneRotateMotorCmd);
        robot.craneExtend.setPower(craneExtendMotorCmd);
        robot.craneWrist.setPosition(craneWristTargetPos);
        robot.craneClaw.setPosition(craneClawTargetPos);

        telemetry.addLine("DO YOU KNOW DA WAY");
//        if(telemetryActivated) {
//      Send telemetry to display on the phone
//        telemetry.addData("leftWheelsMotorCmd ", "%.2f", leftWheelsMotorCmd);
//        telemetry.addData("rightWheelsMotorCmd", "%.2f", rightWheelsMotorCmd);
//        telemetry.addData("armMotorCmd        ", "%.2f", armMotorCmd);
//        telemetry.addData("Left Wheel Pos ", robot.leftDrive.getCurrentPosition());
//        telemetry.addData("Right Wheel Pos", robot.rightDrive.getCurrentPosition());
        telemetry.addData("Arm Pos sin       ", Math.sin(armAngle));
        telemetry.addData("Arm Power       ", armMotorCmd);
            telemetry.addData("Arm Pos        ", robot.armRotate.getCurrentPosition()).setRetained(false);
        telemetry.addData("Arm Speed        ", armSpeed);
        telemetry.addData("Arm Max       ", armMax);
            telemetry.addData("Wrist Pos        ", craneWristTargetPos).setRetained(false);
            telemetry.addData("Claw Pos        ", craneClawTargetPos).setRetained(false);
//      telemetry.addData("Arm Speed      ", armSpeed);
            telemetry.addData("Crane Rotate Pos ", craneRotateCurrentPos).setRetained(false);
            telemetry.addData("Crane Rotate Target Pos ", craneRotateTargetPos).setRetained(false);
            telemetry.addData("Crane Rotate Motor Cmd ", craneRotateMotorCmd).setRetained(false);
            telemetry.addData("Crane Extend Pos ", craneExtendCurrentPos).setRetained(false);
            telemetry.addData("Flicker Finger Target Pos ", flickerFingerTargetPos).setRetained(false);
            telemetry.addData("Flicker Arm Target Pos ", flickerArmTargetPos).setRetained(false);
            telemetry.addData("gamepad2.dpad_down ", gamepad2.dpad_down).setRetained(false);
            telemetry.addData("Gyro ", getHeading()).setRetained(false);
//        telemetry.addData("Crane Extend Target Pos ", craneExtendTargetPos);
//        telemetry.addData("Crane Extnd Motor Cmd ", craneExtendMotorCmd);
//        telemetry.addData("Red Color      ", robot.colorSensor.red());
//        telemetry.addData("Red Color      ", robot.colorSensor.blue());

//        telemetry.addData("collectorFinger1Pos      ", collectorFinger1Pos);
//        telemetry.addData("collectorFinger2Pos      ", collectorFinger2Pos);
//        } else {
//            telemetry.addLine("Press START to show telemetry");
//            telemetry.clearAll();
//            telemetry.clear();
//        }
        telemetry.update();
    }
    private double getHeading ()
    {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop ()
    {
    }

}
