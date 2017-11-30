package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Final.Hardware;
import org.firstinspires.ftc.teamcode.Final.PinkPD;
import org.firstinspires.ftc.teamcode.Final.Presets;
import org.firstinspires.ftc.teamcode.Testing.Subsystems.ArmControl;
import org.firstinspires.ftc.teamcode.Testing.Subsystems.BaseControl;
import org.firstinspires.ftc.teamcode.Testing.Subsystems.ClawControl;
import org.firstinspires.ftc.teamcode.Testing.Subsystems.CollectorControl;
import org.firstinspires.ftc.teamcode.Testing.Subsystems.CraneControl;

@SuppressWarnings({"FieldCanBeLocal", "UnusedDeclaration"})

@TeleOp (name = "TeleopTest")
public class TeleopTest extends OpMode {

        //-----------------------------Declare OpMode Members-----------------------------//
        private Hardware robot = new Hardware();
        private boolean collectorRotateButtonWasntAlreadyPressed;
        private boolean craneClawOpenButtonWasntAlreadyPressed;
        private double previousArmPos = 0;
        private double previousCraneRotatePos = 0;
        private double armSpeed = 0;
        private double craneRotateSpeed = 0;
        private double leftJoystick1 = -gamepad1.left_stick_y;
        private double rightJoystick1 = -gamepad1.right_stick_y;
        private double leftJoystick2 = -gamepad2.left_stick_y;
        private double rightJoystick2 = -gamepad2.right_stick_y;
        private double leftWheelsMotorCmd, rightWheelsMotorCmd, armMotorCmd;
        private double craneRotateMotorCmd, craneExtendMotorCmd;
        private double collectorFinger1Pos = Presets.COLLECTOR_FINGER_GRAB_POS;
        private double collectorFinger2Pos = Presets.COLLECTOR_FINGER_GRAB_POS;
        private double collectorRotatePos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
        private double armTargetPos = 0;
        private double armCurrentPos = 0;
        private double craneRotateTargetPos = 0;
        private double craneRotateCurrentPos = 0;
        private double craneExtendTargetPos = 0;
        private double craneExtendCurrentPos = 0;
        private double craneWristTargetPos = Presets.CRANE_WRIST_STOW_POS;
        private double craneClawTargetPos = Presets.CRANE_CLAW_CLOSE_POS;

        @Override
        public void init () {
            robot.init(hardwareMap);

            robot.flickerArm.setPosition(Presets.FLICKER_ARM_STOW_POS);
            robot.flickerFinger.setPosition(Presets.FLICKER_FINGER_STOW_POS);

            telemetry.addData("Say", "Hello Driver");
        }

        @Override
        public void loop () {
            armCurrentPos = robot.armRotate.getCurrentPosition();
            armSpeed = armCurrentPos - previousArmPos;
            previousArmPos = armCurrentPos;

            craneExtendCurrentPos = robot.craneExtend.getCurrentPosition();

            craneRotateCurrentPos = robot.craneRotate.getCurrentPosition();
            craneRotateSpeed = craneRotateCurrentPos - previousCraneRotatePos;
            previousCraneRotatePos = craneRotateCurrentPos;

            collectorFinger1Pos = Presets.COLLECTOR_FINGER_GRAB_POS;
            collectorFinger2Pos = Presets.COLLECTOR_FINGER_GRAB_POS;

            //------------------------------Base Control------------------------------//
            if (gamepad1.left_trigger > 0.5) {
                BaseControl.leftDriveFullSpeed(leftJoystick1);
                BaseControl.rightDriveFullSpeed(rightJoystick1);
            }
            else {
                BaseControl.leftDriveHalfSpeed(leftJoystick1);
                BaseControl.rightDriveHalfSpeed(rightJoystick1);
            }

            //----------------------------Collector Control---------------------------//
            if (gamepad2.b) {
                CollectorControl.collectorFinger1Score();
                CollectorControl.collectorFinger2Score();
            }
            if (gamepad1.right_bumper) {
                CollectorControl.collectorOpenBottomFinger();
            }
            if (gamepad1.left_bumper) {
                CollectorControl.collectorOpenTopFinger();
            }
            if ((gamepad2.right_bumper) && (collectorRotateButtonWasntAlreadyPressed)) {
                CollectorControl.collectorSetToggleFalse();
                CollectorControl.collectorToggleRotate();
            }
            if (!gamepad2.right_bumper) {
                CollectorControl.collectorSetToggleTrue();
            }

            //-------------------------------Arm Control------------------------------//
            if (gamepad2.a) {
                ArmControl.armGoToCollect();
            }
            else if (gamepad2.y) {
                ArmControl.armGoToHighScore();
            }
            else if (gamepad2.x) {
                ArmControl.armGoToLowScore();
            }
            // Manual arm movement
            if ((rightJoystick2 > 0.1) || (rightJoystick2 < -0.1)) {
                ArmControl.armManualControl(rightJoystick2);
            }

            //------------------------------Claw Control------------------------------//
            if ((gamepad2.right_trigger > 0.5) && (craneClawOpenButtonWasntAlreadyPressed)) {
                ClawControl.clawSetToggleFalse();
                ClawControl.clawToggleGrab();
            }
            if (gamepad2.right_trigger < 0.5) {
                ClawControl.clawSetToggleTrue();
            }
            if ((gamepad2.left_stick_y > 0.1) || (leftJoystick2 < -0.1)) {
                ClawControl.clawWristManualControl(leftJoystick2);
            }

            //------------------------------Crane Control-----------------------------//
            if (gamepad2.dpad_up) {
                CraneControl.craneRotateUp();
            }
            else if (gamepad2.dpad_down) {
                CraneControl.craneRotateDown();
            }
            if (gamepad2.dpad_right) {
               CraneControl.craneExtend();
            }
            else if (gamepad2.dpad_left) {
                CraneControl.craneExtend();
            }
            else {
                CraneControl.craneHold();
            }
            if (gamepad2.left_bumper) {
                CraneControl.craneWristScore();
            }
            if (gamepad2.left_trigger > 0.5) {
                CraneControl.craneScoreSequenceWrist();
                CraneControl.craneScoreSequenceRotate();
                CraneControl.craneScoreSequenceClaw();
            }

            //------------------------Limit Position and Power------------------------//
            armTargetPos = Range.clip(armTargetPos, Presets.COLLECTOR_ARM_MIN_POS, Presets.COLLECTOR_ARM_MAX_POS);
            if (armTargetPos <= Presets.COLLECTOR_ARM_LOW_SCORE_POS) {
                armMotorCmd = PinkPD.getMotorCmd(0.01, 0.01, armTargetPos - armCurrentPos, armSpeed);
            }
            else {
                armMotorCmd = PinkPD.getMotorCmd(0.02, 0.02, armTargetPos - armCurrentPos, armSpeed);
            }
            armMotorCmd = Range.clip(armMotorCmd, -0.1, 0.8);
            craneRotateTargetPos = Range.clip(craneRotateTargetPos, Presets.CRANE_ROTATE_MIN_POS, Presets.CRANE_ROTATE_MAX_POS);
            craneExtendTargetPos = Range.clip(craneExtendTargetPos, Presets.CRANE_EXTEND_MIN_POS, Presets.CRANE_EXTEND_MAX_POS);
            craneRotateMotorCmd = PinkPD.getMotorCmd(0.03, 0.02, craneRotateTargetPos - craneRotateCurrentPos, craneRotateSpeed);
            craneWristTargetPos = Range.clip(craneWristTargetPos, Presets.CRANE_WRIST_MIN_POS, Presets.CRANE_WRIST_MAX_POS);


            //------------------------Set Powers and Positions------------------------//
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
        }
}