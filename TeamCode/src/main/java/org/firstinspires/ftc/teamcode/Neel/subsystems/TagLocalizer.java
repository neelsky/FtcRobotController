package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

public class TagLocalizer {
    public static double MAX_STALENESS_MS = 100;
    public static double MIN_TAG_GAP_IN = 3;
    public static double MIN_SCALE = 0.6;
    public static double MAX_SCALE = 1.6;

    private final Limelight3A limelight;
    private String lastResult = "no tags yet";

    public TagLocalizer(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public String getLastResult() {
        return lastResult;
    }

    public Pose locate() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) {
            lastResult = "no fresh tags";
            return null;
        }

        List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (blueEquivalent(tag.getFiducialId()) >= 0) {
                tags.add(tag);
            }
        }

        LLResultTypes.FiducialResult first = null;
        LLResultTypes.FiducialResult second = null;
        double widest = 0;
        for (int i = 0; i < tags.size(); i++) {
            for (int j = i + 1; j < tags.size(); j++) {
                int a = tags.get(i).getFiducialId();
                int b = tags.get(j).getFiducialId();
                if (group(a) != group(b)) {
                    continue;
                }
                double gap = Math.abs(FieldMap.fieldXOffsetInches(blueEquivalent(a))
                        - FieldMap.fieldXOffsetInches(blueEquivalent(b)));
                if (gap > widest) {
                    widest = gap;
                    first = tags.get(i);
                    second = tags.get(j);
                }
            }
        }
        if (first == null || widest < MIN_TAG_GAP_IN) {
            lastResult = "need 2 tags on the same cell (seeing " + tags.size() + ")";
            return null;
        }

        double pitch = Math.toRadians(Vision.CAMERA_PITCH_DEG);
        double[] c1 = cameraInches(first);
        double[] c2 = cameraInches(second);
        if (c1[2] <= 1 || c2[2] <= 1) {
            lastResult = "no 3D data: turn on Full 3D Targeting";
            return null;
        }

        double fwd1 = c1[2] * Math.cos(pitch) + c1[1] * Math.sin(pitch);
        double fwd2 = c2[2] * Math.cos(pitch) + c2[1] * Math.sin(pitch);
        double measuredGap = Math.hypot(fwd2 - fwd1, c2[0] - c1[0]);
        double scale = widest / Math.max(measuredGap, 1e-6);
        if (scale < MIN_SCALE || scale > MAX_SCALE) {
            lastResult = String.format("bad tag reading (scale %.2f)", scale);
            return null;
        }

        double height1 = scale * (c1[2] * Math.sin(pitch) - c1[1] * Math.cos(pitch)) + Vision.CAMERA_HEIGHT_IN;
        double height2 = scale * (c2[2] * Math.sin(pitch) - c2[1] * Math.cos(pitch)) + Vision.CAMERA_HEIGHT_IN;
        boolean cellUp = (height1 + height2) / 2 > (FieldMap.UP_TAGS_HEIGHT_IN + FieldMap.DOWN_TAGS_HEIGHT_IN) / 2;

        double[] field1 = tagField(first.getFiducialId(), cellUp);
        double[] field2 = tagField(second.getFiducialId(), cellUp);

        double f1 = scale * fwd1 + Vision.CAMERA_FORWARD_OF_CENTER_IN;
        double l1 = -(scale * c1[0] + Vision.CAMERA_RIGHT_OF_CENTER_IN);
        double f2 = scale * fwd2 + Vision.CAMERA_FORWARD_OF_CENTER_IN;
        double l2 = -(scale * c2[0] + Vision.CAMERA_RIGHT_OF_CENTER_IN);

        double heading = Math.atan2(field2[1] - field1[1], field2[0] - field1[0]) - Math.atan2(l2 - l1, f2 - f1);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double x = ((field1[0] - (f1 * cos - l1 * sin)) + (field2[0] - (f2 * cos - l2 * sin))) / 2;
        double y = ((field1[1] - (f1 * sin + l1 * cos)) + (field2[1] - (f2 * sin + l2 * cos))) / 2;

        lastResult = String.format("tags %d+%d, cell %s", first.getFiducialId(), second.getFiducialId(), cellUp ? "up" : "down");
        return new Pose(x, y, Math.atan2(Math.sin(heading), Math.cos(heading)));
    }

    private static int blueEquivalent(int id) {
        if (id >= 38 && id <= 45) return id;
        if (id >= 30 && id <= 37) return id + 8;
        return -1;
    }

    private static int group(int id) {
        return (id - 30) / 4;
    }

    private static double[] tagField(int id, boolean cellUp) {
        int blue = blueEquivalent(id);
        double fromPivot = cellUp ? FieldMap.UP_TAGS_FROM_PIVOT_IN : FieldMap.DOWN_TAGS_FROM_PIVOT_IN;
        double x = FieldMap.PIVOT_X + FieldMap.fieldXOffsetInches(blue);
        double y = FieldMap.clusterOf(blue) == 1 ? FieldMap.PIVOT_Y + fromPivot : FieldMap.PIVOT_Y - fromPivot;
        if (id < 38) {
            return new double[]{FieldMap.FIELD - x, FieldMap.FIELD - y};
        }
        return new double[]{x, y};
    }

    private static double[] cameraInches(LLResultTypes.FiducialResult tag) {
        Position pos = tag.getTargetPoseCameraSpace().getPosition();
        return new double[]{pos.unit.toInches(pos.x), pos.unit.toInches(pos.y), pos.unit.toInches(pos.z)};
    }
}
