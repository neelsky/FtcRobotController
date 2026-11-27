package org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.aim;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap;
import org.firstinspires.ftc.teamcode.Neel.subsystems.PoseKeeper;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.PoseMemory;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.FIELD;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.wrapDegrees;

public class OdometryAim {
    public static double AUTO_POSE_MAX_AGE_S = 120;
    public static double SAVED_FILE_MAX_AGE_S = 120;
    public static double KEPT_MATCH_IN = 24;
    public static double ROBOT_HALF_IN = 9;
    public static double VELOCITY_WINDOW_S = 0.25;
    public static double CELL_SMOOTHING = 0.3;
    public static double CELL_JUMP_IN = 12;
    public static double CELL_IGNORE_IN = 0.75;
    public static double FORGET_CELL_S = 1.0;
    public static int MEDIAN_FRAMES = 5;
    public static double CELL_CENTER_OUT_TRIM_IN = 0;
    public static double CELL_CENTER_SIDE_TRIM_IN = 7;

    private final Follower follower;
    private final ElapsedTime clock = new ElapsedTime();
    private Alliance alliance = Alliance.BLUE;
    private boolean poseKnown = false;
    private String positionSource = "none";

    private final double[][] history = new double[64][4];
    private int historyCount = 0;
    private int historyNext = 0;
    private double vx = 0;
    private double vy = 0;
    private double omegaDeg = 0;

    private boolean cellKnown = false;
    private double cellX = 0;
    private double cellY = 0;
    private int cell = -1;
    private double cellSeenAt = -1e9;
    private final java.util.ArrayList<double[]> recent = new java.util.ArrayList<>();

    public OdometryAim(HardwareMap hardwareMap) {
        Pose kept = PoseKeeper.readPinpoint(hardwareMap);
        PoseKeeper.Saved saved = PoseKeeper.loadFile();
        boolean memoryFresh = PoseMemory.getPose() != null && PoseMemory.secondsSinceSave() < AUTO_POSE_MAX_AGE_S;
        boolean savedFresh = saved != null && saved.ageSeconds < SAVED_FILE_MAX_AGE_S;
        if (memoryFresh) {
            alliance = PoseMemory.getAlliance();
        } else if (saved != null) {
            alliance = saved.alliance;
        }
        Pose last = memoryFresh ? PoseMemory.getPose() : savedFresh ? saved.pose : null;

        follower = Constants.createFollower(hardwareMap);
        if (kept != null && last != null && Math.hypot(kept.getX() - last.getX(), kept.getY() - last.getY()) <= KEPT_MATCH_IN) {
            setStart(kept, "kept by the Pinpoint");
        } else if (kept == null && last != null) {
            setStart(last, memoryFresh ? "carried over from the auto" : "saved file");
        } else {
            follower.setStartingPose(new Pose(0, 0, 0));
        }
    }

    public OdometryAim(HardwareMap hardwareMap, Alliance startAlliance, Pose startPose) {
        alliance = startAlliance;
        follower = Constants.createFollower(hardwareMap);
        setStart(startPose, "auto start spot");
    }

    private void setStart(Pose pose, String source) {
        follower.setStartingPose(pose);
        poseKnown = true;
        positionSource = source;
    }

    public void update(boolean followerAlreadyUpdated, double dt) {
        if (!followerAlreadyUpdated) {
            follower.updatePose();
        }
        Pose p = follower.getPose();
        double now = clock.seconds();
        double h = Math.toDegrees(p.getHeading());
        history[historyNext] = new double[]{now, p.getX(), p.getY(), h};
        historyNext = (historyNext + 1) % history.length;
        historyCount = Math.min(historyCount + 1, history.length);
        double[] old = null;
        for (int i = 1; i < historyCount; i++) {
            double[] sample = history[(historyNext - 1 - i + history.length) % history.length];
            old = sample;
            if (now - sample[0] >= VELOCITY_WINDOW_S) {
                break;
            }
        }
        if (old != null && now - old[0] > 1e-3) {
            double span = now - old[0];
            vx = (p.getX() - old[1]) / span;
            vy = (p.getY() - old[2]) / span;
            omegaDeg = wrapDegrees(h - old[3]) / span;
        }

        if (cellKnown && poseKnown && cell != sideCell(p) && clock.seconds() - cellSeenAt > FORGET_CELL_S) {
            cellKnown = false;
        }
        if (poseKnown) {
            PoseKeeper.save(p, alliance);
        }
    }

    public double[] tagsSeen(double tagsX, double tagsY, int seenCell, boolean upCell) {
        double tagsFromPivot = upCell ? FieldMap.UP_TAGS_FROM_PIVOT_IN : FieldMap.DOWN_TAGS_FROM_PIVOT_IN;
        double out = FieldMap.CELL_AIM_FROM_PIVOT_IN - tagsFromPivot;
        double sign = outSign(seenCell);
        cellSeen(tagsX, tagsY + sign * out, seenCell);
        return withTrim(tagsX, tagsY + sign * out, seenCell);
    }

    private double outSign(int whichCell) {
        double sign = whichCell == 1 ? 1 : -1;
        return alliance == Alliance.RED ? -sign : sign;
    }

    private double[] withTrim(double x, double y, int whichCell) {
        double sign = outSign(whichCell);
        return new double[]{x + sign * CELL_CENTER_SIDE_TRIM_IN, y + sign * CELL_CENTER_OUT_TRIM_IN};
    }

    private void cellSeen(double rawX, double rawY, int seenCell) {
        if (!cellKnown || seenCell != cell) {
            recent.clear();
        }
        recent.add(new double[]{rawX, rawY});
        while (recent.size() > Math.max(1, MEDIAN_FRAMES)) {
            recent.remove(0);
        }
        double x = median(0);
        double y = median(1);
        boolean jump = !cellKnown || seenCell != cell || Math.hypot(x - cellX, y - cellY) > CELL_JUMP_IN;
        if (jump) {
            cellX = x;
            cellY = y;
        } else if (Math.hypot(x - cellX, y - cellY) > CELL_IGNORE_IN) {
            cellX += CELL_SMOOTHING * (x - cellX);
            cellY += CELL_SMOOTHING * (y - cellY);
        }
        cellKnown = true;
        cell = seenCell;
        cellSeenAt = clock.seconds();
    }

    private double median(int index) {
        double[] values = new double[recent.size()];
        for (int i = 0; i < values.length; i++) {
            values[i] = recent.get(i)[index];
        }
        java.util.Arrays.sort(values);
        int middle = values.length / 2;
        return values.length % 2 == 1 ? values[middle] : (values[middle - 1] + values[middle]) / 2;
    }

    public double secondsSinceCellSeen() {
        return clock.seconds() - cellSeenAt;
    }

    public double[] aimPoint() {
        if (cellKnown) {
            return withTrim(cellX, cellY, cell);
        }
        if (poseKnown) {
            int which = sideCell(follower.getPose());
            double[] map = mapCell(which);
            return withTrim(map[0], map[1], which);
        }
        return null;
    }

    public String aimPointSource() {
        if (cellKnown) {
            return clock.seconds() - cellSeenAt < 0.5 ? "LIMELIGHT (cell seen now)" : "ODOMETRY (cell where the Limelight last saw it)";
        }
        if (poseKnown) {
            return "ODOMETRY (cell from the field map, Limelight hasn't seen it yet)";
        }
        return "UNKNOWN: press gamepad 2 Y to look for the tags, or gamepad 1 BACK on the start spot";
    }

    public Pose poseAhead(double seconds) {
        Pose p = follower.getPose();
        return new Pose(p.getX() + vx * seconds, p.getY() + vy * seconds,
                p.getHeading() + Math.toRadians(omegaDeg * seconds));
    }

    public static double[] pivotOnField(Pose robot, double pivotForwardIn, double pivotLeftIn) {
        double h = robot.getHeading();
        return new double[]{
                robot.getX() + pivotForwardIn * Math.cos(h) - pivotLeftIn * Math.sin(h),
                robot.getY() + pivotForwardIn * Math.sin(h) + pivotLeftIn * Math.cos(h)
        };
    }

    private int sideCell(Pose robot) {
        double blueY = alliance == Alliance.RED ? FIELD - robot.getY() : robot.getY();
        return blueY > FieldMap.PIVOT_Y ? 1 : 0;
    }

    private double[] mapCell(int which) {
        double x = FieldMap.PIVOT_X;
        double y = FieldMap.PIVOT_Y + (which == 1 ? 1 : -1) * FieldMap.CELL_AIM_FROM_PIVOT_IN;
        if (alliance == Alliance.RED) {
            return new double[]{FIELD - x, FIELD - y};
        }
        return new double[]{x, y};
    }

    public void resyncToStartSpot() {
        double x = FieldMap.CENTER + ROBOT_HALF_IN + 1;
        double y = FIELD - ROBOT_HALF_IN;
        Pose spot = alliance == Alliance.RED
                ? new Pose(FIELD - x, FIELD - y, Math.toRadians(90))
                : new Pose(x, y, Math.toRadians(-90));
        follower.setPose(spot);
        historyCount = 0;
        vx = 0;
        vy = 0;
        omegaDeg = 0;
        cellKnown = false;
        recent.clear();
        poseKnown = true;
        positionSource = "re-synced on the start spot";
    }

    public void setAlliance(Alliance newAlliance) {
        if (newAlliance != alliance) {
            alliance = newAlliance;
            cellKnown = false;
        }
    }

    public boolean isStill(double maxSpeedInPerS, double maxTurnDegPerS) {
        return Math.hypot(vx, vy) < maxSpeedInPerS && Math.abs(omegaDeg) < maxTurnDegPerS;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Follower getFollower() {
        return follower;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public boolean isPoseKnown() {
        return poseKnown;
    }

    public String getPositionSource() {
        return positionSource;
    }
}
