package org.firstinspires.ftc.teamcode.Neel.subsystems.map;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * FieldMapMetric contain the location of key elements on the field, in meters.
 *
 * Origin is the center of the field.
 * It models the field from the perspective of the drive team, regardless of team color.
 * Looking at the field, left and right is X axis, right being positive.
 * Far and close to be Y axis, far to be positive.
 */
public class FieldMapMetric {

    public static final double MAX_X = 3.65;
    public static final double MIN_X = -3.65;
    public static final double MAX_Y = 3.65;
    public static final double MIN_Y = -3.65;

    public static final Position RIGHT_TARGET = new Position(DistanceUnit.METER, 0.395, -0.324, 0, 0);
    public static final Position LEFT_TARGET = new Position(DistanceUnit.METER, -0.395, -0.324, 0, 0);

    public static final Position RIGHT_CORNER = new Position(DistanceUnit.METER, 3.465, -3.4, 0,0);
    public static final Position LEFT_CORNER = new Position(DistanceUnit.METER, -3.465, -3.4, 0,0);

    public Pose2D test;
    public Position getTargetFromPos(double x, double y) {
        if (x > 0) {
            return RIGHT_TARGET;
        }
        return LEFT_TARGET;
    }

    public static double viewerOffset(int tag) {
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

    public static double toInch(double meter) {
        return DistanceUnit.INCH.fromMeters(meter);
    }

}
