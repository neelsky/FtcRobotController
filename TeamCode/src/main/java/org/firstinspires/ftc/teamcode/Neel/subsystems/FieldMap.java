package org.firstinspires.ftc.teamcode.Neel.subsystems;

public class FieldMap {
    public static final double FIELD = 141.5;
    public static final double CENTER = FIELD / 2;
    public static final double PIVOT_X = CENTER + 12.75;
    public static final double PIVOT_Y = CENTER;
    public static final double CELL_AIM_FROM_PIVOT_IN = 16.1;
    public static final double UP_TAGS_FROM_PIVOT_IN = 13.85;
    public static final double DOWN_TAGS_FROM_PIVOT_IN = 10.85;
    public static final double UP_TAGS_HEIGHT_IN = 48.48;
    public static final double DOWN_TAGS_HEIGHT_IN = 34.22;

    public static int clusterOf(int blueTag) {
        return blueTag <= 41 ? 0 : 1;
    }

    public static double viewerOffsetInches(int blueTag) {
        double cm;
        switch (blueTag) {
            case 38: cm = 17; break;
            case 39: cm = 7; break;
            case 40: cm = -7; break;
            case 41: cm = -17; break;
            case 42: cm = 17; break;
            case 43: cm = 7; break;
            case 44: cm = -7; break;
            case 45: cm = -17; break;
            default: cm = 0; break;
        }
        return cm / 2.54;
    }

    public static double fieldXOffsetInches(int blueTag) {
        double cm;
        switch (blueTag) {
            case 38: cm = -17; break;
            case 39: cm = -7; break;
            case 40: cm = 7; break;
            case 41: cm = 17; break;
            case 42: cm = 17; break;
            case 43: cm = 7; break;
            case 44: cm = -7; break;
            case 45: cm = -17; break;
            default: cm = 0; break;
        }
        return cm / 2.54;
    }

    public static double wrapDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
