package org.firstinspires.ftc.teamcode.Neel.subsystems.map;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * FieldMapMetric contain the location of key elements on the field, in meters.
 *
 * Origin is the center of the field.
 * It models the field from the perspective of the drive team, regardless of team color.
 * Looking at the field, left and right is X axis, right being positive.
 * Far and close to be Y axis, far to be positive.
 */
public class FieldMap {

    public static final double MAX_X = 72;
    public static final double MIN_X = -72;
    public static final double MAX_Y = 72;
    public static final double MIN_Y = -72;

    public static final Position RIGHT_TARGET = new Position(DistanceUnit.INCH, 15.56, -12.76, 0, 0);
    public static final Position LEFT_TARGET = new Position(DistanceUnit.INCH, -15.56, -12.76, 0, 0);

    public static final Pose RIGHT_CORNER_POSE = new Pose( 64.75, -62.4, AngleUnit.Pif);
    public static final Pose LEFT_CORNER_POSE = new Pose(-64.75, -62.4, AngleUnit.Pif);

    public Pose2D test;
    public static Position getTargetFromPos(double x, double y) {
        if (Math.abs(x) < Math.abs(RIGHT_TARGET.x) && Math.abs(y) < Math.abs(RIGHT_TARGET.y)) {
            return null;
        }
        if (x > 0) {
            return RIGHT_TARGET;
        }
        return LEFT_TARGET;
    }

    public static double viewerOffsetInCM(int tag) {
        int blueTag = tag < 38? tag + 8 : tag;
        double cm;
        switch (blueTag) {
            case 38: cm = 0.17; break;
            case 39: cm = 0.07; break;
            case 40: cm = -0.07; break;
            case 41: cm = -0.17; break;
            case 42: cm = 0.17; break;
            case 43: cm = 0.07; break;
            case 44: cm = -0.07; break;
            case 45: cm = -0.17; break;
            default: cm = 0; break;
        }
        return cm;
    }


}
