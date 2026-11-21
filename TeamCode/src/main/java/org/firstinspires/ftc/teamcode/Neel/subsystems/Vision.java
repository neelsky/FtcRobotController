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
    public static double CELL_LOCK_MS = 700;
    public static double TAG_SWITCH_RATIO = 1.5;
    public static double MIN_TAG_SPREAD_IN2 = 10;
    public static double SCALE_SMOOTHING = 0.1;
    public static double MIN_SCALE = 0.6;
    public static double MAX_SCALE = 1.6;
    public static int PIPELINE = 0;
    public static double CAMERA_X_SIGN = 1;
    public static double CAMERA_Y_SIGN = 1;
    public static double PITCH_LEARN_MIN_MOVE_IN = 6;
    public static double PITCH_LEARN_SMOOTHING = 0.2;
    public static double PITCH_LEARN_MIN_FROM_SIGHT_DEG = 12;
    public static double PITCH_LEARN_MAX_AGE_MS = 2000;
    public static int PITCH_LEARN_MIN_SAMPLES = 9;
    public static double PITCH_LEARN_MAX_SPREAD_DEG = 8;
    public static double PITCH_LEARN_UPDATE_DEG = 1.5;
    public static double PITCH_LEARN_MIN_DEG = 5;
    public static double PITCH_LEARN_MAX_DEG = 85;
    private static String cameraFixMessage = "";
    public static boolean LEARN_CAMERA = false;
    public static final String CAMERA_CHECK_FILE = "/sdcard/FIRST/glitch_camera_check.txt";

    private final Limelight3A limelight;
    private Alliance alliance;
    private final ElapsedTime lastSeenTimer = new ElapsedTime();
    private final ElapsedTime faceTimer = new ElapsedTime();

    private boolean tagVisible = false;
    private int tagId = -1;
    private int tagsUsed = 0;
    private double smoothScale = Double.NaN;
    private double distanceToTarget = -1;
    private double angleToTarget = 0;
    private String aimingAt = "nothing";
    private boolean freshTag = false;
    private boolean fresh3d = false;
    private double flatDistance = -1;
    private double riseInches = -1;
    private double frameAgeMs = 1e9;
    private int bestCluster = -1;
    private String rawTagIds = "";
    private boolean sawOurTag = false;
    private int lockedCluster = -1;
    private double rightLength = 0;
    private boolean measuredScale = false;
    private double scaledFlat = -1;
    private double scaledRise = -1;
    private double scaledAngle = 0;
    private double riseIfYFlipped = -1;
    private final ElapsedTime lockTimer = new ElapsedTime();
    private final ElapsedTime learnClock = new ElapsedTime();
    private final java.util.HashMap<Integer, double[]> pitchRefs = new java.util.HashMap<>();
    private final java.util.HashMap<Integer, double[]> pitchSmooth = new java.util.HashMap<>();
    private final double[] pitchSamples = new double[21];
    private int pitchSampleNext = 0;
    private int pitchSampleCount = 0;

    private boolean hasFaceAngle = false;
    private int faceCluster = -1;
    private double faceAngleWorld = 0;

    public Vision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        loadCameraCheck();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    public static void loadCameraCheck() {
        if (!LEARN_CAMERA) {
            return;
        }
        try (java.io.BufferedReader reader = new java.io.BufferedReader(new java.io.FileReader(new java.io.File(CAMERA_CHECK_FILE)))) {
            String[] parts = reader.readLine().trim().split("\\s+");
            CAMERA_Y_SIGN = Double.parseDouble(parts[1]) < 0 ? -1 : 1;
            CAMERA_X_SIGN = CAMERA_Y_SIGN;
            if (parts.length > 2) {
                double pitch = Double.parseDouble(parts[2]);
                if (pitch >= PITCH_LEARN_MIN_DEG && pitch <= PITCH_LEARN_MAX_DEG) {
                    CAMERA_PITCH_DEG = pitch;
                }
            }
        } catch (Exception ignored) {
        }
    }

    public static void saveCameraCheck() {
        try (java.io.FileWriter writer = new java.io.FileWriter(new java.io.File(CAMERA_CHECK_FILE), false)) {
            writer.write((CAMERA_X_SIGN < 0 ? "-1" : "1") + " " + (CAMERA_Y_SIGN < 0 ? "-1" : "1") + " "
                    + String.format(java.util.Locale.US, "%.2f", CAMERA_PITCH_DEG));
        } catch (Exception ignored) {
        }
    }

    public static String getCameraFixMessage() {
        return cameraFixMessage;
    }

    private void learnPitch(LLResultTypes.FiducialResult tag) {
        if (!LEARN_CAMERA) {
            return;
        }
        if (tag.getTargetPoseCameraSpace() == null) {
            return;
        }
        double[] v = tagPositionInches(tag);
        if (!(v[2] > 1) || Double.isNaN(v[0]) || Double.isNaN(v[1])) {
            return;
        }
        double now = learnClock.milliseconds();
        int id = tag.getFiducialId();
        double[] smooth = pitchSmooth.get(id);
        if (smooth == null || now - smooth[3] > PITCH_LEARN_MAX_AGE_MS) {
            smooth = new double[]{v[0], v[1], v[2], now};
            pitchSmooth.put(id, smooth);
            pitchRefs.put(id, new double[]{v[0], v[1], v[2], now});
            return;
        }
        for (int i = 0; i < 3; i++) {
            smooth[i] += PITCH_LEARN_SMOOTHING * (v[i] - smooth[i]);
        }
        smooth[3] = now;
        double[] ref = pitchRefs.get(id);
        if (ref == null || now - ref[3] > PITCH_LEARN_MAX_AGE_MS) {
            pitchRefs.put(id, new double[]{smooth[0], smooth[1], smooth[2], now});
            return;
        }
        double dy = smooth[1] - ref[1];
        double dz = smooth[2] - ref[2];
        if (Math.hypot(dy, dz) < PITCH_LEARN_MIN_MOVE_IN || Math.abs(dz) < 1e-6) {
            return;
        }
        pitchRefs.put(id, new double[]{smooth[0], smooth[1], smooth[2], now});
        double estimate = Math.toDegrees(Math.atan(dy / dz));
        double lineOfSight = Math.toDegrees(Math.atan(smooth[1] / smooth[2]));
        double apart = Math.abs(estimate - lineOfSight);
        apart = Math.min(apart, 180 - apart);
        if (apart < PITCH_LEARN_MIN_FROM_SIGHT_DEG) {
            return;
        }
        pitchSamples[pitchSampleNext] = estimate;
        pitchSampleNext = (pitchSampleNext + 1) % pitchSamples.length;
        pitchSampleCount = Math.min(pitchSampleCount + 1, pitchSamples.length);
        if (pitchSampleCount < PITCH_LEARN_MIN_SAMPLES) {
            return;
        }
        double[] sorted = java.util.Arrays.copyOf(pitchSamples, pitchSampleCount);
        java.util.Arrays.sort(sorted);
        double median = sorted[sorted.length / 2];
        double spread = sorted[(sorted.length * 3) / 4] - sorted[sorted.length / 4];
        boolean mostlyBelow = sorted[(sorted.length * 3) / 4] < 0;
        if (spread > PITCH_LEARN_MAX_SPREAD_DEG && !mostlyBelow) {
            return;
        }
        if (mostlyBelow && median <= -PITCH_LEARN_MIN_DEG) {
            CAMERA_Y_SIGN = -CAMERA_Y_SIGN;
            CAMERA_X_SIGN = CAMERA_Y_SIGN;
            resetTracking();
            if (spread <= PITCH_LEARN_MAX_SPREAD_DEG && -median <= PITCH_LEARN_MAX_DEG) {
                CAMERA_PITCH_DEG = -median;
            }
            pitchSampleCount = 0;
            pitchSampleNext = 0;
            pitchRefs.clear();
            pitchSmooth.clear();
            saveCameraCheck();
            cameraFixMessage = String.format(java.util.Locale.US, "camera is upside down: fixed automatically (tilt %.0f°)", CAMERA_PITCH_DEG);
        } else if (spread <= PITCH_LEARN_MAX_SPREAD_DEG && median >= PITCH_LEARN_MIN_DEG && median <= PITCH_LEARN_MAX_DEG
                && Math.abs(median - CAMERA_PITCH_DEG) > PITCH_LEARN_UPDATE_DEG) {
            CAMERA_PITCH_DEG = median;
            saveCameraCheck();
            cameraFixMessage = String.format(java.util.Locale.US, "camera tilt measured: %.0f°", CAMERA_PITCH_DEG);
        }
    }

    public static double tx(LLResultTypes.FiducialResult tag) {
        return CAMERA_X_SIGN * tag.getTargetXDegrees();
    }

    public void setAlliance(Alliance newAlliance) {
        if (newAlliance == alliance) {
            return;
        }
        alliance = newAlliance;
        resetTracking();
    }

    public void resetTracking() {
        lockedCluster = -1;
        tagId = -1;
        tagVisible = false;
        tagsUsed = 0;
        distanceToTarget = -1;
        hasFaceAngle = false;
        faceCluster = -1;
    }

    public void stop() {
        limelight.stop();
    }

    public void update(double heading, int targetCluster) {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> seen = new ArrayList<>();
        freshTag = false;
        fresh3d = false;
        measuredScale = false;
        rawTagIds = "";
        sawOurTag = false;
        if (result != null && result.isValid()) {
            frameAgeMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                rawTagIds += (rawTagIds.isEmpty() ? "" : ", ") + tag.getFiducialId();
                if (Alliance.BLUE.isHiveTag(tag.getFiducialId()) || Alliance.RED.isHiveTag(tag.getFiducialId())) {
                    learnPitch(tag);
                }
                int blue = alliance.toBlueTag(tag.getFiducialId());
                if (blue >= 0) {
                    sawOurTag = true;
                }
                if (blue >= 0 && (targetCluster < 0 || FieldMap.clusterOf(blue) == targetCluster)) {
                    seen.add(tag);
                }
            }
        }

        if (targetCluster < 0 && lockedCluster >= 0) {
            List<LLResultTypes.FiducialResult> locked = new ArrayList<>();
            for (LLResultTypes.FiducialResult tag : seen) {
                if (FieldMap.clusterOf(alliance.toBlueTag(tag.getFiducialId())) == lockedCluster) {
                    locked.add(tag);
                }
            }
            if (!locked.isEmpty()) {
                seen = locked;
            } else if (lockTimer.milliseconds() < CELL_LOCK_MS) {
                seen = locked;
            } else {
                lockedCluster = -1;
            }
        }

        LLResultTypes.FiducialResult best = null;
        LLResultTypes.FiducialResult previous = null;
        for (LLResultTypes.FiducialResult tag : seen) {
            if (best == null || tag.getTargetArea() > best.getTargetArea()) {
                best = tag;
            }
            if (tag.getFiducialId() == tagId) {
                previous = tag;
            }
        }
        if (previous != null && best != null && best.getTargetArea() < previous.getTargetArea() * TAG_SWITCH_RATIO) {
            best = previous;
        }
        if (best != null) {
            lockedCluster = FieldMap.clusterOf(alliance.toBlueTag(best.getFiducialId()));
            lockTimer.reset();
        }

        if (best != null) {
            tagId = best.getFiducialId();
            bestCluster = FieldMap.clusterOf(alliance.toBlueTag(tagId));
            freshTag = true;
            double[] p = tagPositionInches(best);
            if (p[2] > 1) {
                double[] middle = findCellMiddle(best, p, seen, heading);
                double pitch = Math.toRadians(CAMERA_PITCH_DEG);
                double xRobot = middle[0] + CAMERA_RIGHT_OF_CENTER_IN;
                double zRobot = middle[2] * Math.cos(pitch) + middle[1] * Math.sin(pitch) + CAMERA_FORWARD_OF_CENTER_IN;
                flatDistance = Math.hypot(xRobot, zRobot);
                riseInches = middle[2] * Math.sin(pitch) - middle[1] * Math.cos(pitch);
                riseIfYFlipped = middle[2] * Math.sin(pitch) + middle[1] * Math.cos(pitch);
                fresh3d = true;
                if (tagsUsed >= 2 && rightLength > 1e-6) {
                    double measured = 1.0 / rightLength;
                    if (measured >= MIN_SCALE && measured <= MAX_SCALE) {
                        smoothScale = Double.isNaN(smoothScale) ? measured : smoothScale + SCALE_SMOOTHING * (measured - smoothScale);
                    }
                }
                if (!Double.isNaN(smoothScale)) {
                    double sx = middle[0] * smoothScale;
                    double sy = middle[1] * smoothScale;
                    double sz = middle[2] * smoothScale;
                    double scaledX = sx + CAMERA_RIGHT_OF_CENTER_IN;
                    double scaledZ = sz * Math.cos(pitch) + sy * Math.sin(pitch) + CAMERA_FORWARD_OF_CENTER_IN;
                    scaledFlat = Math.hypot(scaledX, scaledZ);
                    scaledRise = sz * Math.sin(pitch) - sy * Math.cos(pitch);
                    scaledAngle = Math.toDegrees(Math.atan2(scaledX, scaledZ)) + AIM_TRIM_DEG;
                    measuredScale = true;
                }
                distanceToTarget = Math.sqrt(middle[0] * middle[0] + middle[1] * middle[1] + middle[2] * middle[2]);
                angleToTarget = Math.toDegrees(Math.atan2(xRobot, zRobot)) + AIM_TRIM_DEG;
                aimingAt = "CELL CENTER (" + tagsUsed + (tagsUsed == 1 ? " tag" : " tags") + ")";
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
                double perInch = (tx(best) - tx(tag)) / (offset - bestOffset);
                tagsUsed = 2;
                aimingAt = "CELL CENTER (no 3D, 2 tags)";
                return tx(best) + bestOffset * perInch + AIM_TRIM_DEG;
            }
        }
        tagsUsed = 0;
        aimingAt = "TAG ONLY: turn on Full 3D";
        return tx(best) + AIM_TRIM_DEG;
    }

    private double[] findCellMiddle(LLResultTypes.FiducialResult best, double[] p,
                                    List<LLResultTypes.FiducialResult> seen, double heading) {
        int bestBlue = alliance.toBlueTag(best.getFiducialId());
        int cluster = FieldMap.clusterOf(bestBlue);
        double bestOffset = FieldMap.viewerOffsetInches(bestBlue);
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);

        double sumOffset = 0;
        double[] sumPos = new double[3];
        List<double[]> positions = new ArrayList<>();
        List<Double> offsets = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : seen) {
            int blue = alliance.toBlueTag(tag.getFiducialId());
            if (FieldMap.clusterOf(blue) != cluster) {
                continue;
            }
            double[] q = tag == best ? p : tagPositionInches(tag);
            if (!(q[2] > 1)) {
                continue;
            }
            double off = FieldMap.viewerOffsetInches(blue);
            positions.add(q);
            offsets.add(off);
            sumOffset += off;
            for (int i = 0; i < 3; i++) {
                sumPos[i] += q[i];
            }
        }
        int n = positions.size();
        double meanOffset = n > 0 ? sumOffset / n : bestOffset;
        double[] meanPos = n > 0 ? new double[]{sumPos[0] / n, sumPos[1] / n, sumPos[2] / n} : p;
        double spread = 0;
        double[] slope = new double[3];
        for (int k = 0; k < n; k++) {
            double d = offsets.get(k) - meanOffset;
            spread += d * d;
            for (int i = 0; i < 3; i++) {
                slope[i] += d * (positions.get(k)[i] - meanPos[i]);
            }
        }

        double[] right = {1, 0, 0};
        double[] base = p;
        double baseOffset = bestOffset;
        if (n >= 2 && spread >= MIN_TAG_SPREAD_IN2) {
            right = new double[]{-slope[0] / spread, -slope[1] / spread, -slope[2] / spread};
            base = meanPos;
            baseOffset = meanOffset;
            rightLength = Math.sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
            double rightForward = right[2] * Math.cos(pitch) + right[1] * Math.sin(pitch);
            faceAngleWorld = Math.toDegrees(Math.atan2(rightForward, right[0])) + heading;
            faceCluster = cluster;
            hasFaceAngle = true;
            faceTimer.reset();
            tagsUsed = n;
        } else if (hasFaceAngle && faceCluster == cluster && faceTimer.milliseconds() < FACE_MEMORY_MS) {
            double a = Math.toRadians(faceAngleWorld - heading);
            right = new double[]{Math.cos(a), Math.sin(a) * Math.sin(pitch), Math.sin(a) * Math.cos(pitch)};
            tagsUsed = 1;
        } else {
            tagsUsed = 1;
        }

        double[] middle = {
                base[0] + baseOffset * right[0],
                base[1] + baseOffset * right[1],
                base[2] + baseOffset * right[2]
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
                CAMERA_X_SIGN * pos.unit.toInches(pos.x) * scale,
                CAMERA_Y_SIGN * pos.unit.toInches(pos.y) * scale,
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

    public boolean hasFreshTag() {
        return freshTag;
    }

    public boolean hasFresh3d() {
        return fresh3d;
    }

    public double getFlatDistance() {
        return flatDistance;
    }

    public double getRise() {
        return riseInches;
    }

    public double getRiseIfYFlipped() {
        return riseIfYFlipped;
    }

    public double getFrameAgeMs() {
        return frameAgeMs;
    }

    public boolean hasMeasuredScale() {
        return measuredScale;
    }

    public double getScaledFlatDistance() {
        return scaledFlat;
    }

    public double getScaledRise() {
        return scaledRise;
    }

    public double getScaledAngle() {
        return scaledAngle;
    }

    public String getRawTagIds() {
        return rawTagIds;
    }

    public boolean sawOurAllianceTag() {
        return sawOurTag;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public int getTargetCluster() {
        return bestCluster;
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
