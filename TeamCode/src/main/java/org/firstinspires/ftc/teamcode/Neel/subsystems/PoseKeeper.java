package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class PoseKeeper {
    public static String FILE_PATH = "/sdcard/FIRST/glitch_pose.txt";
    public static double SAVE_EVERY_S = 0.5;
    public static double ORIGIN_RADIUS_IN = 0.5;
    public static double FIELD_MARGIN_IN = 12;

    private static long lastSaveMillis = 0;

    public static class Saved {
        public final Pose pose;
        public final Alliance alliance;
        public final double ageSeconds;

        Saved(Pose pose, Alliance alliance, double ageSeconds) {
            this.pose = pose;
            this.alliance = alliance;
            this.ageSeconds = ageSeconds;
        }
    }

    public static Pose readPinpoint(HardwareMap hardwareMap) {
        try {
            GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                    Constants.localizerConstants.hardwareMapName);
            pinpoint.update();
            Pose pose = PoseConverter.pose2DToPose(pinpoint.getPosition(), PedroCoordinates.INSTANCE);
            if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
                return null;
            }
            if (Math.hypot(pose.getX(), pose.getY()) < ORIGIN_RADIUS_IN) {
                return null;
            }
            if (!onField(pose)) {
                return null;
            }
            return pose;
        } catch (Exception e) {
            return null;
        }
    }

    public static boolean onField(Pose pose) {
        double lo = -FIELD_MARGIN_IN;
        double hi = FieldMap.FIELD + FIELD_MARGIN_IN;
        return pose.getX() > lo && pose.getX() < hi && pose.getY() > lo && pose.getY() < hi;
    }

    public static void save(Pose pose, Alliance alliance) {
        PoseMemory.save(pose, alliance);
        long now = System.currentTimeMillis();
        if (now - lastSaveMillis < SAVE_EVERY_S * 1000) {
            return;
        }
        lastSaveMillis = now;
        try (FileWriter writer = new FileWriter(new File(FILE_PATH), false)) {
            writer.write(pose.getX() + " " + pose.getY() + " " + pose.getHeading() + " " + alliance + " " + now);
        } catch (Exception ignored) {
        }
    }

    public static Saved loadFile() {
        try (BufferedReader reader = new BufferedReader(new FileReader(new File(FILE_PATH)))) {
            String[] parts = reader.readLine().trim().split(" ");
            Pose pose = new Pose(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]), Double.parseDouble(parts[2]));
            Alliance alliance = Alliance.valueOf(parts[3]);
            double age = (System.currentTimeMillis() - Long.parseLong(parts[4])) / 1000.0;
            if (age < 0 || !onField(pose)) {
                return null;
            }
            return new Saved(pose, alliance, age);
        } catch (Exception e) {
            return null;
        }
    }
}
