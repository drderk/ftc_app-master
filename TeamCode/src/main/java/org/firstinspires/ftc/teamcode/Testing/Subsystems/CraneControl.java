package org.firstinspires.ftc.teamcode.Testing.Subsystems;

import org.firstinspires.ftc.teamcode.Final.Hardware;
import org.firstinspires.ftc.teamcode.Final.Presets;


public class CraneControl {

    static Hardware robot = new Hardware();
    static double craneExtendMotorCmd;
    static double craneRotateTargetPos;
    static double craneWristTargetPos;
    static double craneClawTargetPos;

    public static double craneRotateUp() {
        craneRotateTargetPos = craneRotateTargetPos + 4;
        return craneRotateTargetPos;
    }

    public static double craneRotateDown() {
        craneRotateTargetPos = craneRotateTargetPos - 2;
        return craneRotateTargetPos;
    }

    public static double craneExtend() {
        if (robot.craneExtend.getCurrentPosition() < Presets.CRANE_EXTEND_MAX_POS) {
            craneExtendMotorCmd = 1.0;
            return craneExtendMotorCmd;
        }
        else {
            craneExtendMotorCmd = 0;
            return craneExtendMotorCmd;
        }
    }

    public static double craneRetract() {
        craneExtendMotorCmd = -0.5;
        return craneExtendMotorCmd;
    }

    public static double craneHold() {
        craneExtendMotorCmd = 0;
        return craneExtendMotorCmd;
    }

    public static double craneWristScore() {
        craneWristTargetPos = Presets.CRANE_WRIST_SCORE_POS;
        return craneWristTargetPos;
    }

    public static double craneScoreSequenceWrist() {
        craneWristTargetPos = Presets.CRANE_WRIST_COLLECT_POS;
        return  craneWristTargetPos;
    }

    public static double craneScoreSequenceRotate() {
        craneRotateTargetPos = Presets.CRANE_ROTATE_COLLECT_POS;
        return craneRotateTargetPos;
    }

    public static double craneScoreSequenceClaw() {
        craneClawTargetPos = Presets.CRANE_CLAW_OPEN_POS;
        return craneClawTargetPos;
    }
}
