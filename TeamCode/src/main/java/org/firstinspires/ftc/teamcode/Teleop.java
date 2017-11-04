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

@TeleOp (name = "Pushbot: Teleop Tank", group = "Pushbot")
public class Teleop extends OpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware

/*    static final double FINGER_CLOSE_POS = -1;
    static final double FINGER_OPEN_POS = 1;
    static final double FINGER_SCORE_POS = 0.5;
    static final double ARM_COLLECT_POS = 0;
    static final double ARM_LOW_SCORE_POS = 30;
    static final double ARM_HIGH_SCORE_POS = 280;
    static final double ARM_MAX_POS = 280;
    static final double ARM_MIN_POS = 0;
    static final double COLLECTOR_UPRIGHT_POS = -1;
    static final double COLLECTOR_INVERTED_POS = 1;
    */
    static boolean collectorRotateButtonWasntAlreadyPressed;
    static double previousArmPos = 0;
    double armSpeed = 0; // Max = 22
    double leftJoystick, rightJoystick;
    double leftWheelsMotorCmd, rightWheelsMotorCmd, armMotorCmd;
    double collectorFinger1Pos = Presets.COLLECTOR_FINGER_GRAB_POSITION;
    double collectorFinger2Pos = Presets.COLLECTOR_FINGER_GRAB_POSITION;
    double collectorRotatePos = Presets.COLLECTOR_ROTATE_UPRIGHT_POSITION;
    double targetArmPos = 0;
    double currentArmPos = 0;

    //Collector Opmode
    Collector collector = new Collector();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

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
        currentArmPos = robot.lift.getCurrentPosition();
        armSpeed =  currentArmPos - previousArmPos;

        // Close the fingers if no other commands are given
        collectorFinger1Pos = Presets.COLLECTOR_FINGER_GRAB_POSITION;
        collectorFinger2Pos = Presets.COLLECTOR_FINGER_GRAB_POSITION;

        // Run wheels in tank mode (The joystick is negative when pushed forward, so negate it)
        leftJoystick = -gamepad1.left_stick_y;
        rightJoystick = -gamepad1.right_stick_y;

        // Scoring finger position
        if (gamepad2.a) {
            collectorFinger1Pos = Presets.COLLECTOR_FINGER_SCORE_POSITION;     // Slightly open to release glyph
            collectorFinger2Pos = Presets.COLLECTOR_FINGER_SCORE_POSITION;     // Slightly open to release glyph
        }

        // Open bottom fingers and lower the tower when collecting
        if (gamepad1.right_bumper) {
            if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POSITION)
                collectorFinger2Pos = Presets.COLLECTOR_FINGER_COLLECT_POSITION;
            else if (collectorRotatePos == Presets.COLLECTOR_ROTATE_INVERTED_POSITION) {
                collectorFinger1Pos = Presets.COLLECTOR_FINGER_COLLECT_POSITION;
            }
            targetArmPos = Presets.COLLECTOR_ARM_COLLECT_POS;
        }

        // Raise the collector slightly to keep it off the floor
        // or high to score
        if (gamepad2.x) {
            targetArmPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
        }
        else if (gamepad2.y) {
            targetArmPos = Presets.COLLECTOR_ARM_HIGH_SCORE_POS;
        }

        // Manual arm movement
        if (gamepad1.dpad_up) {
            targetArmPos = targetArmPos + 1;
        } else if (gamepad1.dpad_down) {
            targetArmPos = targetArmPos - 1;
        }

        // Toggle the collector rotate position
        if ((gamepad2.right_bumper)&&(collectorRotateButtonWasntAlreadyPressed)) {
            collectorRotateButtonWasntAlreadyPressed = false;
            if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POSITION) {
                collectorRotatePos = Presets.COLLECTOR_ROTATE_INVERTED_POSITION;
            }
            else {
                collectorRotatePos = Presets.COLLECTOR_ROTATE_UPRIGHT_POSITION;
            }
        }
        if (gamepad2.right_bumper == false){
            collectorRotateButtonWasntAlreadyPressed = true;
        }

        // Limit position and power
        targetArmPos = Range.clip(targetArmPos, ARM_MIN_POS, ARM_MAX_POS);
        armMotorCmd = PinkPD.getMotorCmd(0.1, 0.01, targetArmPos - currentArmPos, armSpeed);
        armMotorCmd = Range.clip(armMotorCmd, 0.001, 0.7);
        leftWheelsMotorCmd = leftJoystick * 0.5;
        rightWheelsMotorCmd = rightJoystick * 0.5;


        // Set powers and positions
        robot.leftDrive.setPower(leftWheelsMotorCmd);
        robot.rightDrive.setPower(rightWheelsMotorCmd);
        robot.lift.setPower(armMotorCmd);
        robot.collect1.setPosition(collectorFinger1Pos);
        robot.collect2.setPosition(collectorFinger2Pos);
        robot.rotate.setPosition(collectorRotatePos);

        // Send telemetry to display on the phone
        telemetry.addData("leftWheelsMotorCmd ", "%.2f", leftWheelsMotorCmd);
        telemetry.addData("rightWheelsMotorCmd", "%.2f", rightWheelsMotorCmd);
        telemetry.addData("armMotorCmd        ", "%.2f", armMotorCmd);

        telemetry.addData("Left Wheel Pos ", robot.leftDrive.getCurrentPosition());
        telemetry.addData("Right Wheel Pos", robot.rightDrive.getCurrentPosition());
        telemetry.addData("Arm Pos        ", robot.lift.getCurrentPosition());
        telemetry.addData("Arm Speed      ", armSpeed);
        telemetry.addData("collectorFinger1Pos      ", collectorFinger1Pos);
        telemetry.addData("collectorFinger2Pos      ", collectorFinger2Pos);
        previousArmPos = currentArmPos;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop ()
    {
    }

}
