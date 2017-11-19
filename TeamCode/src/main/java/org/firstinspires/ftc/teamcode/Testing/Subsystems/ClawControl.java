package org.firstinspires.ftc.teamcode.Testing.Subsystems;

import org.firstinspires.ftc.teamcode.Final.Presets;


public class ClawControl {

    static double craneClawTargetPos;
    static double craneWristTargetPos;
    static boolean craneClawOpenButtonWasntAlreadyPressed;

    public static double clawToggleGrab() {
        if (craneClawTargetPos == Presets.CRANE_CLAW_OPEN_POS) {
            craneClawTargetPos = Presets.CRANE_CLAW_CLOSE_POS;
            return craneClawTargetPos;
        }
        else {
            craneClawTargetPos = Presets.CRANE_CLAW_OPEN_POS;
            return craneClawTargetPos;
        }
    }

    public static double clawWristManualControl(double leftJoystick2) {
        craneWristTargetPos = craneWristTargetPos - (0.02 * leftJoystick2);
        return craneWristTargetPos;
    }

    public static boolean clawSetToggleTrue() {
        craneClawOpenButtonWasntAlreadyPressed = true;
        return craneClawOpenButtonWasntAlreadyPressed;
    }

    public static boolean clawSetToggleFalse() {
        craneClawOpenButtonWasntAlreadyPressed = false;
        return craneClawOpenButtonWasntAlreadyPressed;
    }
}