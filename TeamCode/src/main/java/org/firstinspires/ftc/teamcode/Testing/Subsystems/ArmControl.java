package org.firstinspires.ftc.teamcode.Testing.Subsystems;

import org.firstinspires.ftc.teamcode.Final.Presets;


public class ArmControl {
    
    static double armTargetPos;
    
    public static double armGoToCollect() {
        armTargetPos = Presets.COLLECTOR_ARM_COLLECT_POS;
        return armTargetPos;
    }
    
    public static double armGoToHighScore() {
        armTargetPos = Presets.COLLECTOR_ARM_HIGH_SCORE_POS;
        return armTargetPos;
    }

    public static double armGoToLowScore() {
        armTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
        return  armTargetPos;
    }

    public static double armManualControl(double rightJoystick2) {
        armTargetPos = armTargetPos - (5.0 * rightJoystick2);
        return armTargetPos;
    }
}