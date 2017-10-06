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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.Hardware;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
public class TankDrive extends OpMode{

    /* Declare OpMode members. */
    Hardware robot       = new Hardware(); // use the class created to define a Pushbot's hardware
    
    double left;
    double right;
    double collectPos = 0;
    double liftPow = 0;
    double rotatePos = 0;
    double liftCPI = 0; //Counts per inch
    
    //positions
    double liftUp = 12 * liftCPI;
    double liftDown = 0 * liftCPI;
    double liftHold = 0;

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

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        
        //collect buttons
        if (gamepad2.left_bumper) {
            collectPos = -1;
        }
        else { //collect neutral
        	collectPos = 0;
        }

        //lift buttons
        if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) { //manual control
            liftPow = -gamepad2.left_stick_y;
            //liftHold = robot.lift.getCurrentPosition();
        }
        else if (gamepad2.dpad_up){ //lift up position
        	//liftPow = PinkPD.getMotorCmd(0.1, 0.001, liftUp - robot.lift.getCurrentPosition()/*insert encoder error*/, 0 /*insert velocity error*/);
        	liftHold = liftPow;
        } 
        else if (gamepad2.dpad_down){ //lift down position
        	//liftPow = PinkPD.getMotorCmd(0.1, 0.001, liftDown - robot.lift.getCurrentPosition()/*insert encoder error*/, 0 /*insert velocity error*/);
        	liftHold = liftPow;

        }
        else { //Holds position
        	//liftPow = PinkPD.getMotorCmd(0.1, 0.001, liftHold - robot.lift.getCurrentPosition()/*insert encoder error*/, 0 /*insert velocity error*/);
        }
        
        //rotate buttons
       /* if (gamepad2.x)
        {
            if (robot.rotate.getPosition() < 0.5) {
                rotatePos = 1;
            }
            else {
                rotatePos = 0;
            }
        }*/

        //sets powers and positions
        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        /*robot.collect.setPosition(collectPos);
        robot.lift.setPower(liftPow);
        robot.rotate.setPosition(rotatePos);*/
        
        //Sends telemetry to the phone
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("Left Encoder", "%d", robot.leftDrive.getCurrentPosition());
        telemetry.addData("Right Encoder", "%d", robot.rightDrive.getCurrentPosition());
        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
