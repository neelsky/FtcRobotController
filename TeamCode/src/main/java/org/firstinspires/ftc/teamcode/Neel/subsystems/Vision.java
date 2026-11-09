package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    public static double TAG_HOLD_MS = 300;
    public static double FACE_MEMORY_MS = 3000;
    public static double REAL_TAG_MM = 82.55;
    public static double LIMELIGHT_MARKER_MM = 101.6;
    public static double CELL_DEPTH_OFFSET_IN = 0;
    public static double CAMERA_PITCH_DEG = 60;
    public static double CAMERA_HEIGHT_IN = 13;
    public static double CAMERA_RIGHT_OF_CENTER_IN = 5.0 / 2.54;
    public static double CAMERA_FORWARD_OF_CENTER_IN = 0;
    public static double AIM_TRIM_DEG = 0;

    private final Limelight3A limelight;
    private final Alliance alliance;
    private final ElapsedTime lastSeenTimer = new ElapsedTime();
    private final ElapsedTime faceTimer = new ElapsedTime();

    private boolean tagVisible = false;
    private int tagId = -1;
    private int tagsUsed = 0;
    private double distanceToTarget = -1;
    private double angleToTarget = 0;
    private String aimingAt = "nothing";

    private boolean hasFaceAngle = false;
    private int faceCluster = -1;
    private double faceAngleWorld = 0;

    public Vision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void update(double heading, int targetCluster) {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> seen = new ArrayList<>();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int blue = alliance.toBlueTag(tag.getFiducialId());
                if (blue >= 0 && (targetCluster < 0 || FieldMap.clusterOf(blue) == targetCluster)) {
                    seen.add(tag);
                }
            }
        }

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult tag : seen) {
            if (best == null || tag.getTargetArea() > best.getTargetArea()) {
                best = tag;
            }
        }

        if (best != null) {
            tagId = best.getFiducialId();
            double[] p = tagPositionInches(best);
            if (p[2] > 1) {
                double[] middle = findCellMiddle(best, p, seen, heading);
                double pitch = Math.toRadians(CAMERA_PITCH_DEG);
                double xRobot = middle[0] + CAMERA_RIGHT_OF_CENTER_IN;
                double zRobot = middle[2] * Math.cos(pitch) + middle[1] * Math.sin(pitch) + CAMERA_FORWARD_OF_CENTER_IN;
                distanceToTarget = Math.sqrt(middle[0] * middle[0] + middle[1] * middle[1] + middle[2] * middle[2]);
                angleToTarget = Math.toDegrees(Math.atan2(xRobot, zRobot)) + AIM_TRIM_DEG;
                aimingAt = "CELL CENTER (" + (tagsUsed == 2 ? "2 tags" : "1 tag") + ")";
            } else {
                angleToTarget = twoTagAngleWithout3d(best, seen);
            }
            tagVisible = true;
            lastSeenTimer.reset();
        } else if (lastSeenTimer.milliseconds() > TAG_HOLD_MS) {
            tagVisible = false;
            tagId = -1;
            tagsUsed = 0;
            distanceToTarget = -1;
        }
        if (!tagVisible) {
            aimingAt = "nothing";
        }
    }

    private double twoTagAngleWithout3d(LLResultTypes.FiducialResult best, List<LLResultTypes.FiducialResult> seen) {
        int bestBlue = alliance.toBlueTag(best.getFiducialId());
        double bestOffset = FieldMap.viewerOffsetInches(bestBlue);
        for (LLResultTypes.FiducialResult tag : seen) {
            int blue = alliance.toBlueTag(tag.getFiducialId());
            double offset = FieldMap.viewerOffsetInches(blue);
            if (tag != best && FieldMap.clusterOf(blue) == FieldMap.clusterOf(bestBlue) && offset != bestOffset) {
                double perInch = (best.getTargetXDegrees() - tag.getTargetXDegrees()) / (offset - bestOffset);
                tagsUsed = 2;
                aimingAt = "CELL CENTER (no 3D, 2 tags)";
                return best.getTargetXDegrees() + bestOffset * perInch + AIM_TRIM_DEG;
            }
        }
        tagsUsed = 0;
        aimingAt = "TAG ONLY: turn on Full 3D";
        return best.getTargetXDegrees() + AIM_TRIM_DEG;
    }

    private double[] findCellMiddle(LLResultTypes.FiducialResult best, double[] p,
                                    List<LLResultTypes.FiducialResult> seen, double heading) {
        int bestBlue = alliance.toBlueTag(best.getFiducialId());
        int cluster = FieldMap.clusterOf(bestBlue);
        double bestOffset = FieldMap.viewerOffsetInches(bestBlue);
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);

        LLResultTypes.FiducialResult partner = null;
        double[] partnerPos = null;
        double widestGap = 0;
        for (LLResultTypes.FiducialResult tag : seen) {
            int blue = alliance.toBlueTag(tag.getFiducialId());
            if (tag == best || FieldMap.clusterOf(blue) != cluster) {
                continue;
            }
            double gap = Math.abs(FieldMap.viewerOffsetInches(blue) - bestOffset);
            double[] q = tagPositionInches(tag);
            if (q[2] > 1 && gap > widestGap) {
                partner = tag;
                partnerPos = q;
                widestGap = gap;
            }
        }

        double[] right = {1, 0, 0};
        if (partner != null) {
            double offsetDiff = FieldMap.viewerOffsetInches(alliance.toBlueTag(partner.getFiducialId())) - bestOffset;
            right = new double[]{
                    (p[0] - partnerPos[0]) / offsetDiff,
                    (p[1] - partnerPos[1]) / offsetDiff,
                    (p[2] - partnerPos[2]) / offsetDiff
            };
            double rightForward = right[2] * Math.cos(pitch) + right[1] * Math.sin(pitch);
            faceAngleWorld = Math.toDegrees(Math.atan2(rightForward, right[0])) + heading;
            faceCluster = cluster;
            hasFaceAngle = true;
            faceTimer.reset();
            tagsUsed = 2;
        } else if (hasFaceAngle && faceCluster == cluster && faceTimer.milliseconds() < FACE_MEMORY_MS) {
            double a = Math.toRadians(faceAngleWorld - heading);
            right = new double[]{Math.cos(a), Math.sin(a) * Math.sin(pitch), Math.sin(a) * Math.cos(pitch)};
            tagsUsed = 1;
        } else {
            tagsUsed = 1;
        }

        double[] middle = {
                p[0] + bestOffset * right[0],
                p[1] + bestOffset * right[1],
                p[2] + bestOffset * right[2]
        };

        double[] up = {0, -Math.cos(pitch), Math.sin(pitch)};
        double[] deeper = {
                up[1] * right[2] - up[2] * right[1],
                up[2] * right[0] - up[0] * right[2],
                up[0] * right[1] - up[1] * right[0]
        };
        double length = Math.sqrt(deeper[0] * deeper[0] + deeper[1] * deeper[1] + deeper[2] * deeper[2]);
        if (length > 1e-6) {
            if (deeper[2] * Math.cos(pitch) + deeper[1] * Math.sin(pitch) < 0) {
                length = -length;
            }
            middle[0] += CELL_DEPTH_OFFSET_IN * deeper[0] / length;
            middle[1] += CELL_DEPTH_OFFSET_IN * deeper[1] / length;
            middle[2] += CELL_DEPTH_OFFSET_IN * deeper[2] / length;
        }
        return middle;
    }

    private double[] tagPositionInches(LLResultTypes.FiducialResult tag) {
        Position pos = tag.getTargetPoseCameraSpace().getPosition();
        double scale = REAL_TAG_MM / LIMELIGHT_MARKER_MM;
        return new double[]{
                pos.unit.toInches(pos.x) * scale,
                pos.unit.toInches(pos.y) * scale,
                pos.unit.toInches(pos.z) * scale
        };
    }

    public String freshHiveTagIds(double maxStalenessMs) {
        LLResult result = limelight.getLatestResult();
        StringBuilder ids = new StringBuilder();
        if (result != null && result.isValid() && result.getStaleness() < maxStalenessMs) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (alliance.isHiveTag(tag.getFiducialId())) {
                    ids.append(tag.getFiducialId()).append(' ');
                }
            }
        }
        return ids.toString().trim();
    }

    public boolean isTagVisible() {
        return tagVisible;
    }

    public double getDistance() {
        return distanceToTarget;
    }

    public double getAngle() {
        return angleToTarget;
    }

    public int getTagsUsed() {
        return tagsUsed;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("tag", tagVisible ? tagId + " (" + tagsUsed + " used)" : "not seen");
        telemetry.addData("aimingAt", aimingAt);
        telemetry.addData("distance", "%.1f in", distanceToTarget);
        telemetry.addData("angle", "%.2f°", angleToTarget);
    }
}
