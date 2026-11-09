package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.pedropathing.geometry.Pose;

public class PoseMemory {
    private static Pose lastPose = null;
    private static Alliance lastAlliance = null;

    public static void save(Pose pose, Alliance alliance) {
        if (pose != null) {
            lastPose = pose;
            lastAlliance = alliance;
        }
    }

    public static Pose getPose() {
        return lastPose;
    }

    public static Alliance getAlliance() {
        return lastAlliance;
    }
}
