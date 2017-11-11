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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.Hardware;

/*
 * This file provides basic Teleop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*/

@TeleOp (name = "Teleop", group = "Pushbot")
public class Teleop extends OpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware

    boolean collectorRotateButtonWasntAlreadyPressed;
    boolean craneClawOpenButtonWasntAlreadyPressed;
    double previousArmPos = 0;
    double previousCraneRotatePos = 0;
    double armSpeed = 0; // Max
    double craneRotateSpeed = 0;
    double leftJoystick, rightJoystick;
    double leftWheelsMotorCmd, rightWheelsMotorCmd, armMotorCmd;
    double craneRotateMotorCmd, craneExtendMotorCmd;
    double collectorFinger1Pos = Presets.COLLECTOR_FINGER_GRAB_POS;
    double collectorFinger2Pos = Presets.COLLECTOR_FINGER_GRAB_POS;
    double collectorRotatePos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
    double armTargetPos = 0;
    double armCurrentPos = 0;
    double craneRotateTargetPos = 0;
    double craneRotateCurrentPos = 0;
    double craneExtendTargetPos = 0;
    double craneExtendCurrentPos = 0;
    double craneWristTargetPos = Presets.CRANE_WRIST_STOW_POS;
    double craneClawTargetPos = Presets.CRANE_CLAW_CLOSE_POS;
    double flickerFingerTargetPos = Presets.FLICKER_FINGER_NEUTRAL_POS;
    double flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
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
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Do up-front calculations for control loops
        armCurrentPos = robot.armRotate.getCurrentPosition();
        armSpeed = armCurrentPos - previousArmPos;
        previousArmPos = armCurrentPos;

        craneExtendCurrentPos = robot.craneExtend.getCurrentPosition();

        craneRotateCurrentPos = robot.craneRotate.getCurrentPosition();
        craneRotateSpeed = craneRotateCurrentPos - previousCraneRotatePos;
        previousCraneRotatePos = craneRotateCurrentPos;

        // Close the fingers if no other commands are given
        collectorFinger1Pos = Presets.COLLECTOR_FINGER_GRAB_POS;
        collectorFinger2Pos = Presets.COLLECTOR_FINGER_GRAB_POS;

        // BASE CONTROL /////////////////////////////////////////////////////
        // Run wheels in tank mode (The joystick is negative when pushed forward, so negate it)
        leftJoystick = -gamepad1.left_stick_y;
        rightJoystick = -gamepad1.right_stick_y;

        if (gamepad1.left_trigger > 0.5) {
            leftWheelsMotorCmd = leftJoystick * 1.0;
            rightWheelsMotorCmd = rightJoystick * 1.0;
        } else {
            leftWheelsMotorCmd = leftJoystick * 0.6;
            rightWheelsMotorCmd = rightJoystick * 0.6;
        }

        // COLLECTOR CONTROL /////////////////////////////////////////////////////
        // Scoring finger position
        if (gamepad2.b) {
            collectorFinger1Pos = Presets.COLLECTOR_FINGER_SCORE_POS;     // Slightly open to release glyph
            collectorFinger2Pos = Presets.COLLECTOR_FINGER_SCORE_POS;     // Slightly open to release glyph
        }
        // Open bottom fingers
        if (gamepad1.right_bumper) {
            if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS) {
                collectorFinger1Pos = Presets.COLLECTOR_FINGER_COLLECT_POS;
            } else {
                collectorFinger2Pos = Presets.COLLECTOR_FINGER_COLLECT_POS;
            }
        }
        // Open top fingers
        if (gamepad1.left_bumper) {
            if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS) {
                collectorFinger2Pos = Presets.COLLECTOR_FINGER_COLLECT_POS;
            } else {
                collectorFinger1Pos = Presets.COLLECTOR_FINGER_COLLECT_POS;
            }
        }

        // Toggle the collector rotate position
        if ((gamepad2.right_bumper) && (collectorRotateButtonWasntAlreadyPressed)) {
            collectorRotateButtonWasntAlreadyPressed = false;
            if (armTargetPos == Presets.COLLECTOR_ARM_COLLECT_POS)
            {
                armTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
            }
            if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS) {
                collectorRotatePos = Presets.COLLECTOR_ROTATE_INVERTED_POS;
            } else {
                collectorRotatePos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
            }
        }
        if (gamepad2.right_bumper == false) {
            collectorRotateButtonWasntAlreadyPressed = true;
        }

        // ARM CONTROL /////////////////////////////////////////////////////
        // Raise the collector slightly to keep it off the floor
        // or high to score
        if (gamepad2.a) {
            armTargetPos = Presets.COLLECTOR_ARM_COLLECT_POS;
        } else if (gamepad2.y) {
            armTargetPos = Presets.COLLECTOR_ARM_HIGH_SCORE_POS;
        } else if (gamepad2.x) {
            armTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
        }

        // Manual arm movement
        if ((gamepad2.right_stick_y > 0.1) || (gamepad2.right_stick_y < -0.1)) {
            armTargetPos = armTargetPos - (5.0 * gamepad2.right_stick_y);
        }

        // CLAW CONTROL /////////////////////////////////////////////////////
        // Toggle the claw finger open/close
        if ((gamepad2.right_trigger > 0.5) && (craneClawOpenButtonWasntAlreadyPressed)) {
            craneClawOpenButtonWasntAlreadyPressed = false;
            if (craneClawTargetPos == Presets.CRANE_CLAW_OPEN_POS) {
                craneClawTargetPos = Presets.CRANE_CLAW_CLOSE_POS;
            } else {
                craneClawTargetPos = Presets.CRANE_CLAW_OPEN_POS;
            }
        }
        if (gamepad2.right_trigger < 0.5) {
            craneClawOpenButtonWasntAlreadyPressed = true;
        }

        if ((gamepad2.left_stick_y > 0.1) || (gamepad2.left_stick_y < -0.1)) {
            craneWristTargetPos = craneWristTargetPos - (0.02 * gamepad2.left_stick_y);
        }
        if (gamepad2.left_trigger > 0.5) {
            craneWristTargetPos = Presets.CRANE_WRIST_SCORE_POS;
        }

        // CRANE CONTROL /////////////////////////////////////////////////////
        if (gamepad2.dpad_up) {
            craneRotateTargetPos = craneRotateTargetPos + 4;
        } else if (gamepad2.dpad_down) {
            craneRotateTargetPos = craneRotateTargetPos - 2;
        }

        if (gamepad2.dpad_right) {
            if (robot.craneExtend.getCurrentPosition() < Presets.CRANE_EXTEND_MAX_POS) {
                craneExtendMotorCmd = 1.0;
            } else {
                craneExtendMotorCmd = 0;
            }
        } else if (gamepad2.dpad_left) {
            craneExtendMotorCmd = -0.5;
        } else {
            craneExtendMotorCmd = 0;
        }
        if (gamepad2.left_bumper) {
            craneWristTargetPos = Presets.CRANE_WRIST_SCORE_POS;
        }
        if (gamepad2.left_trigger > 0.5) {
            craneWristTargetPos = Presets.CRANE_WRIST_COLLECT_POS;
            craneRotateTargetPos = Presets.CRANE_ROTATE_COLLECT_POS;
            craneClawTargetPos = Presets.CRANE_CLAW_OPEN_POS;
        }
        // Limit position and power
        armTargetPos = Range.clip(armTargetPos, Presets.COLLECTOR_ARM_MIN_POS, Presets.COLLECTOR_ARM_MAX_POS);
        if (armTargetPos <= Presets.COLLECTOR_ARM_LOW_SCORE_POS) {
            armMotorCmd = PinkPD.getMotorCmd(0.01, 0.01, armTargetPos - armCurrentPos, armSpeed);
        } else {
            armMotorCmd = PinkPD.getMotorCmd(0.02, 0.02, armTargetPos - armCurrentPos, armSpeed);
        }
        armMotorCmd = Range.clip(armMotorCmd, -0.1, 0.8);
        craneRotateTargetPos = Range.clip(craneRotateTargetPos, Presets.CRANE_ROTATE_MIN_POS, Presets.CRANE_ROTATE_MAX_POS);
        craneExtendTargetPos = Range.clip(craneExtendTargetPos, Presets.CRANE_EXTEND_MIN_POS, Presets.CRANE_EXTEND_MAX_POS);
        craneRotateMotorCmd = PinkPD.getMotorCmd(0.03, 0.02, craneRotateTargetPos - craneRotateCurrentPos, craneRotateSpeed);
        //       craneExtendMotorCmd = PinkPD.getMotorCmd(0.02, 0, craneExtendTargetPos - craneExtendCurrentPos, 0);
        //       craneExtendMotorCmd = Range.clip(craneExtendMotorCmd, -0.4, 0.4);
        craneWristTargetPos = Range.clip(craneWristTargetPos, Presets.CRANE_WRIST_MIN_POS, Presets.CRANE_WRIST_MAX_POS);

/*        if (gamepad1.y)
        {
            flickerArmTargetPos = Presets.FLICKER_ARM_STOW_POS;
        }
        else if (gamepad1.a)
        {
            flickerArmTargetPos = Presets.FLICKER_ARM_DEPLOY_POS;
        }

        if (gamepad1.b)
        {
            //flickerFingerTargetPos += 0.02;
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
*/
        //flickerFingerTargetPos = Range.clip(flickerFingerTargetPos, -1.0, 1.0);
        // Set powers and positions
        robot.leftDrive.setPower(leftWheelsMotorCmd);
        robot.rightDrive.setPower(rightWheelsMotorCmd);
        robot.armRotate.setPower(armMotorCmd);
        robot.collectorFinger1.setPosition(collectorFinger1Pos);
        robot.collectorFinger2.setPosition(collectorFinger2Pos);
        robot.collectorRotate.setPosition(collectorRotatePos);
        robot.craneRotate.setPower(craneRotateMotorCmd);
        robot.craneExtend.setPower(craneExtendMotorCmd);
        robot.craneWrist.setPosition(craneWristTargetPos);
        robot.craneClaw.setPosition(craneClawTargetPos);

        // Send telemetry to display on the phone
//        telemetry.addData("leftWheelsMotorCmd ", "%.2f", leftWheelsMotorCmd);
//        telemetry.addData("rightWheelsMotorCmd", "%.2f", rightWheelsMotorCmd);
//        telemetry.addData("armMotorCmd        ", "%.2f", armMotorCmd);

//        telemetry.addData("Left Wheel Pos ", robot.leftDrive.getCurrentPosition());
//        telemetry.addData("Right Wheel Pos", robot.rightDrive.getCurrentPosition());
        telemetry.addData("Arm Pos        ", robot.armRotate.getCurrentPosition());
        telemetry.addData("Wrist Pos        ", craneWristTargetPos);
        telemetry.addData("Claw Pos        ", craneClawTargetPos);
//        telemetry.addData("Arm Speed      ", armSpeed);
        telemetry.addData("Crane Rotate Pos ", craneRotateCurrentPos);
        telemetry.addData("Crane Rotate Target Pos ", craneRotateTargetPos);
        telemetry.addData("Crane Rotate Motor Cmd ", craneRotateMotorCmd);
        telemetry.addData("Crane Extend Pos ", craneExtendCurrentPos);
        telemetry.addData("Flicker Finger Target Pos ", flickerFingerTargetPos);
        telemetry.addData("Flicker Arm Target Pos ", flickerArmTargetPos);
        telemetry.addData("gamepad2.dpad_down", gamepad2.dpad_down);
//        telemetry.addData("Crane Extend Target Pos ", craneExtendTargetPos);
//        telemetry.addData("Crane Extnd Motor Cmd ", craneExtendMotorCmd);
//        telemetry.addData("Red Color      ", robot.colorSensor.red());
//        telemetry.addData("Red Color      ", robot.colorSensor.blue());

//        telemetry.addData("collectorFinger1Pos      ", collectorFinger1Pos);
//        telemetry.addData("collectorFinger2Pos      ", collectorFinger2Pos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop ()
    {
    }

}
