package org.firstinspires.ftc.teamcode.Testing.Subsystems;

import org.firstinspires.ftc.teamcode.Final.Presets;


public class CollectorControl {

    static double collectorFinger1Pos;
    static double collectorFinger2Pos;
    static double collectorRotatePos;
    static double armTargetPos;
    static boolean collectorRotateButtonWasntAlreadyPressed;

    public static double collectorFinger1Score() {
        collectorFinger1Pos = Presets.COLLECTOR_FINGER_SCORE_POS;
        return collectorFinger1Pos;
    }

    public static double collectorFinger2Score() {
        collectorFinger2Pos = Presets.COLLECTOR_FINGER_SCORE_POS;
        return collectorFinger2Pos;
    }

    public static double collectorOpenBottomFinger() {
        if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS){
            collectorFinger1Pos = Presets.COLLECTOR_FINGER_COLLECT_POS;
            return collectorFinger1Pos;
        }
        else {
            collectorFinger2Pos = Presets.COLLECTOR_FINGER_COLLECT_POS;
            return collectorFinger2Pos;
        }
    }

    public static double collectorOpenTopFinger() {
        if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS) {
            collectorFinger2Pos = Presets.COLLECTOR_FINGER_COLLECT_POS;
            return collectorFinger2Pos;
        }
        else {
            collectorFinger1Pos = Presets.COLLECTOR_FINGER_COLLECT_POS;
            return collectorFinger1Pos;
        }
    }

    public static double collectorToggleRotate() {
        if (armTargetPos == Presets.COLLECTOR_ARM_COLLECT_POS) {
            armTargetPos = Presets.COLLECTOR_ARM_LOW_SCORE_POS;
            return armTargetPos;
        }
        if (collectorRotatePos == Presets.COLLECTOR_ROTATE_UPRIGHT_POS) {
            collectorRotatePos = Presets.COLLECTOR_ROTATE_INVERTED_POS;
            return collectorRotatePos;
        }
        else {
            collectorRotatePos = Presets.COLLECTOR_ROTATE_UPRIGHT_POS;
            return collectorRotatePos;
        }
    }

    public static boolean collectorSetToggleTrue() {
        collectorRotateButtonWasntAlreadyPressed = true;
        return collectorRotateButtonWasntAlreadyPressed;
    }

    public static boolean collectorSetToggleFalse() {
        collectorRotateButtonWasntAlreadyPressed = false;
        return collectorRotateButtonWasntAlreadyPressed;
    }
}
