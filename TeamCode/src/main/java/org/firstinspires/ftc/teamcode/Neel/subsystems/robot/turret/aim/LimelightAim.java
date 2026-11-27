package org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.aim;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap;

import java.util.ArrayList;
import java.util.List;

public class LimelightAim {
    public static int PIPELINE = 0;
    public static double MAX_AGE_MS = 150;
    public static boolean ROBOT_SPACE_Y_IS_RIGHT = true;
    public static double DISTANCE_SCALE = 1.0;
    public static double ROW_MEMORY_S = 30;
    public static double TAG_BIAS_LEARN = 0.1;

    private final Limelight3A limelight;
    private Alliance alliance;

    private boolean seen = false;
    private double forwardIn = 0;
    private double leftIn = 0;
    private double upIn = 0;
    private int cell = -1;
    private int tagCount = 0;
    private double ageMs = 0;
    private String ids = "";
    private double centerTxDeg = 0;
    private double distanceIn = 0;
    private double rowFieldAngle = 0;
    private int rowCell = -1;
    private long rowSeenAtNanos = 0;
    private List<double[]> mineTags = new ArrayList<>();
    private final double[] tagBias = new double[8];
    private double lastPivotF = 0;
    private double lastPivotL = 0;

    public LimelightAim(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public void update(double robotHeadingRad, double turretDeg, double pivotForwardIn, double pivotLeftIn) {
        seen = false;
        tagCount = 0;
        ids = "";
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }
        ageMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        if (ageMs > MAX_AGE_MS) {
            return;
        }

        List<double[]> tags = new ArrayList<>();
        int[] perCell = new int[2];
        StringBuilder seenIds = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (!alliance.isHiveTag(id)) {
                continue;
            }
            Pose3D pose = tag.getTargetPoseRobotSpace();
            if (pose == null) {
                continue;
            }
            Position p = pose.getPosition();
            int blueTag = alliance.toBlueTag(id);
            int tagCell = FieldMap.clusterOf(blueTag);
            double forward = p.unit.toInches(p.x) * DISTANCE_SCALE;
            double left = p.unit.toInches(p.y) * DISTANCE_SCALE * (ROBOT_SPACE_Y_IS_RIGHT ? -1 : 1);
            double up = p.unit.toInches(p.z) * DISTANCE_SCALE;
            double fieldX = FieldMap.fieldXOffsetInches(blueTag) * (alliance == Alliance.RED ? -1 : 1);
            tags.add(new double[]{tagCell, forward, left, up, FieldMap.viewerOffsetInches(blueTag), tag.getTargetXDegrees(), fieldX, blueTag - 38});
            perCell[tagCell]++;
            seenIds.append(id).append(' ');
        }
        if (tags.isEmpty()) {
            return;
        }
        cell = perCell[0] == 0 ? 1 : perCell[1] == 0 ? 0 : nearerCell(tags);
        ids = seenIds.toString().trim();

        List<double[]> mine = new ArrayList<>();
        for (double[] t : tags) {
            if ((int) t[0] == cell) {
                mine.add(t);
            }
        }
        double frameToField = robotHeadingRad + Math.toRadians(turretDeg);
        double dirF = 0;
        double dirL = 0;
        boolean haveDir = false;
        if (mine.size() >= 2) {
            double[] a = mine.get(0);
            double[] b = mine.get(0);
            for (double[] t : mine) {
                if (t[4] < a[4]) a = t;
                if (t[4] > b[4]) b = t;
            }
            double gap = b[4] - a[4];
            if (gap > 1) {
                dirF = (b[1] - a[1]) / gap;
                dirL = (b[2] - a[2]) / gap;
                double length = Math.hypot(dirF, dirL);
                if (length > 1e-6) {
                    dirF /= length;
                    dirL /= length;
                    haveDir = true;
                    rowFieldAngle = Math.atan2(dirL, dirF) + frameToField;
                    rowCell = cell;
                    rowSeenAtNanos = System.nanoTime();
                }
            }
        }
        if (!haveDir) {
            boolean remembered = rowCell == cell && (System.nanoTime() - rowSeenAtNanos) / 1e9 < ROW_MEMORY_S;
            if (remembered) {
                double a = rowFieldAngle - frameToField;
                dirF = Math.cos(a);
                dirL = Math.sin(a);
            } else {
                double[] t = mine.get(0);
                double seeF = t[1] - pivotForwardIn;
                double seeL = t[2] - pivotLeftIn;
                double length = Math.hypot(seeF, seeL);
                if (length > 1e-6) {
                    dirF = -seeL / length;
                    dirL = seeF / length;
                }
            }
        }
        double sumF = 0;
        double sumL = 0;
        double sumU = 0;
        for (double[] t : mine) {
            sumF += t[1] - t[4] * dirF;
            sumL += t[2] - t[4] * dirL;
            sumU += t[3];
        }
        tagCount = mine.size();
        forwardIn = sumF / tagCount;
        leftIn = sumL / tagCount;
        upIn = sumU / tagCount;
        distanceIn = Math.hypot(forwardIn - pivotForwardIn, leftIn - pivotLeftIn);
        centerTxDeg = centerTx(mine);
        mineTags = mine;
        lastPivotF = pivotForwardIn;
        lastPivotL = pivotLeftIn;
        seen = true;
    }

    private double centerTx(List<double[]> mine) {
        double sumTx = 0;
        double sumOffset = 0;
        double[] low = mine.get(0);
        double[] high = mine.get(0);
        for (double[] t : mine) {
            sumTx += t[5];
            sumOffset += t[4];
            if (t[4] < low[4]) low = t;
            if (t[4] > high[4]) high = t;
        }
        double meanTx = sumTx / mine.size();
        double meanOffset = sumOffset / mine.size();
        double degPerInch;
        if (high[4] - low[4] > 1) {
            degPerInch = (high[5] - low[5]) / (high[4] - low[4]);
        } else {
            degPerInch = -Math.toDegrees(1.0 / Math.max(distanceIn, 12));
        }
        return meanTx - meanOffset * degPerInch;
    }

    public double[] tagsMiddleOnField(double cameraYawDeg, double pivotX, double pivotY) {
        int n = mineTags.size();
        double[] range = new double[n];
        double mean = 0;
        for (int i = 0; i < n; i++) {
            double[] t = mineTags.get(i);
            range[i] = Math.hypot(t[1] - lastPivotF, t[2] - lastPivotL);
            mean += range[i] - tagBias[(int) t[7]];
        }
        mean /= n;
        double sumX = 0;
        double sumY = 0;
        for (int i = 0; i < n; i++) {
            double[] t = mineTags.get(i);
            int slot = (int) t[7];
            double corrected = range[i] - tagBias[slot];
            double bearing = Math.toRadians(cameraYawDeg - t[5]);
            sumX += pivotX + corrected * Math.cos(bearing) - t[6];
            sumY += pivotY + corrected * Math.sin(bearing);
            if (n >= 2) {
                tagBias[slot] += TAG_BIAS_LEARN * ((range[i] - mean) - tagBias[slot]);
            }
        }
        return new double[]{sumX / n, sumY / n};
    }

    private int nearerCell(List<double[]> tags) {
        double[] dist = new double[2];
        int[] count = new int[2];
        for (double[] t : tags) {
            int c = (int) t[0];
            dist[c] += Math.hypot(t[1], t[2]);
            count[c]++;
        }
        return dist[1] / count[1] < dist[0] / count[0] ? 1 : 0;
    }

    public String freshHiveTagIds(double maxAgeMs) {
        LLResult result = limelight.getLatestResult();
        StringBuilder found = new StringBuilder();
        if (result != null && result.isValid() && result.getStaleness() < maxAgeMs) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (alliance.isHiveTag(tag.getFiducialId())) {
                    found.append(tag.getFiducialId()).append(' ');
                }
            }
        }
        return found.toString().trim();
    }

    public boolean sees() {
        return seen;
    }

    public double getForwardIn() {
        return forwardIn;
    }

    public double getLeftIn() {
        return leftIn;
    }

    public double getCenterTxDeg() {
        return centerTxDeg;
    }

    public double getDistanceIn() {
        return distanceIn;
    }

    public double getUpIn() {
        return upIn;
    }

    public boolean isUpCell() {
        return upIn > (FieldMap.UP_TAGS_HEIGHT_IN + FieldMap.DOWN_TAGS_HEIGHT_IN) / 2;
    }

    public int getCell() {
        return cell;
    }

    public int getTagCount() {
        return tagCount;
    }

    public double getAgeMs() {
        return ageMs;
    }

    public String getIds() {
        return ids;
    }

    public void stop() {
        limelight.stop();
    }
}
