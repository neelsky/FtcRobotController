package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.pedropathing.geometry.Pose;

public class PoseMemory {
    private static Pose lastPose = null;
    private static Alliance lastAlliance = null;
    private static long savedAtNanos = 0;

    public static void save(Pose pose, Alliance alliance) {
        if (pose != null) {
            lastPose = pose;
            lastAlliance = alliance;
            savedAtNanos = System.nanoTime();
        }
    }

    public static Pose getPose() {
        return lastPose;
    }

    public static Alliance getAlliance() {
        return lastAlliance;
    }

    public static double secondsSinceSave() {
        if (lastPose == null) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - savedAtNanos) / 1e9;
    }
}
