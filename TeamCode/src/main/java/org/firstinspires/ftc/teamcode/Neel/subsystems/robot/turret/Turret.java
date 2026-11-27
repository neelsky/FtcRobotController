package org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.aim.LimelightAim;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.aim.OdometryAim;

import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.wrapDegrees;

public class Turret {
    public static double TURRET_ZERO_POS = 0.5;
    public static double TURRET_DEGREES_PER_UNIT = 305;
    public static double TURRET_LIMIT_DEG = 150;
    public static double TURRET_MIN_POS = 0;
    public static double TURRET_MAX_POS = 1;
    public static boolean TURRET_HIGHER_IS_LEFT = false;
    public static double TURRET_ZERO_FROM_FRONT_DEG = 180;
    public static double PIVOT_FORWARD_IN = -9.74;
    public static double PIVOT_LEFT_IN = 0;
    public static double AIM_TRIM_DEG = 0;

    public static double SPEED_DEG_PER_S = 350;
    public static double PROFILE_MAX_DEG_PER_S = 450;
    public static double PROFILE_ACCEL_DEG_PER_S2 = 6000;
    public static double READY_DEG = 3;
    public static double LAG_S = 0.1;

    public static double HOLD_DEG = 1.5;
    public static double HOLD_CONFIRM_S = 0.3;
    public static double STILL_SPEED_IN_PER_S = 3;
    public static double STILL_TURN_DEG_PER_S = 5;
    public static double MOVING_SPEED_IN_PER_S = 6;
    public static double MOVING_TURN_DEG_PER_S = 12;

    public static double CAMERA_SETTLE_S = 0.3;
    public static double CAMERA_MAX_SPEED_IN_PER_S = 12;
    public static double CAMERA_MAX_TURN_DEG_PER_S = 30;
    public static double CAMERA_AGREE_DEG = 4;
    public static double SEARCH_SPEED_DEG_PER_S = 60;
    public static double QUICK_LOOK_IF_UNSEEN_S = 3.0;

    private final Servo servo;
    private final OdometryAim odometry;
    private final LimelightAim limelight;
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double commandDeg = 0;
    private double sentDeg = 0;
    private double sentSpeed = 0;
    private double estimateDeg = 0;
    private double lastMoveAt = 0;
    private double holdSince = -1;
    private final double[] angleTimes = new double[64];
    private final double[] angleValues = new double[64];
    private int angleNext = 0;
    private int angleCount = 0;
    private boolean still = false;

    private boolean aimKnown = false;
    private boolean aimNow = false;
    private boolean canReach = true;
    private boolean cameraAgrees = true;
    private double distanceToCell = -1;
    private String aimSource = "none";
    private int searchStep = 0;
    private boolean searchWas = false;
    private boolean visionOnly;
    private static final double sweepSpeed = 42;
    private static final double tagLossGracePeriod = 0.65;
    private double lastTagAt = Double.NEGATIVE_INFINITY;

    public Turret(HardwareMap hardwareMap) {
        odometry = new OdometryAim(hardwareMap);
        servo = hardwareMap.get(Servo.class, "turret");
        limelight = new LimelightAim(hardwareMap, odometry.getAlliance());
    }

    public Turret(HardwareMap hardwareMap, Alliance alliance, Pose startPose) {
        odometry = new OdometryAim(hardwareMap, alliance, startPose);
        servo = hardwareMap.get(Servo.class, "turret");
        limelight = new LimelightAim(hardwareMap, alliance);
    }

    public void initLoop(boolean pickBlue, boolean pickRed, boolean resync) {
        if (pickBlue || pickRed) {
            odometry.setAlliance(pickBlue ? Alliance.BLUE : Alliance.RED);
            limelight.setAlliance(odometry.getAlliance());
        }
        if (resync) {
            odometry.resyncToStartSpot();
        }
        double dt = nextDt();
        odometry.update(false, dt);
        commandDeg = 0;
        moveServo(dt);
        if (visionOnly) {
            updateLimelight();
        } else {
            readCamera();
        }
    }

    public void enableVisionOnlyAiming() {
        visionOnly = true;
    }

    public void start() {
        loopTimer.reset();
    }

    public void update(boolean resync, double turnStickX, boolean shooting, boolean searchButton) {
        step(false, resync, searchButton);
    }

    public void updateWhileFollowing(boolean shooting) {
        step(true, false, false);
    }

    private void step(boolean followerAlreadyUpdated, boolean resync, boolean searchButton) {
        if (resync) {
            odometry.resyncToStartSpot();
        }
        double dt = nextDt();
        odometry.update(followerAlreadyUpdated, dt);
        if (visionOnly) {
            aimFromLiveTag(dt);
            moveServo(dt);
            return;
        }
        readCamera();
        if (searchButton && !searchWas) {
            searchStep = 1;
        }
        searchWas = searchButton;
        if (searchStep > 0) {
            search(dt);
        } else {
            aim();
        }
        moveServo(dt);
    }

    private double nextDt() {
        double dt = Math.max(1e-3, Math.min(0.1, loopTimer.seconds()));
        loopTimer.reset();
        return dt;
    }

    private void aimFromLiveTag(double dt) {
        updateLimelight();
        double now = clock.seconds();
        aimKnown = limelight.sees();
        if (!aimKnown) {
            distanceToCell = -1;
            if (now - lastTagAt < tagLossGracePeriod) {
                commandDeg = sentDeg;
                aimSource = "holding for a live tag";
            } else {
                aimSource = "sweeping for a live tag";
                sweep(dt, sweepSpeed);
            }
            return;
        }

        lastTagAt = now;
        searchStep = 0;
        aimSource = "live Limelight tag";
        distanceToCell = limelight.getDistanceIn();
        double error = AIM_TRIM_DEG - limelight.getCenterTxDeg();
        cameraAgrees = Math.abs(error) <= READY_DEG;
        double frameAngle = angleAt(now - limelight.getAgeMs() / 1000.0);
        double target = frameAngle + error;
        canReach = Math.abs(target) <= TURRET_LIMIT_DEG;
        commandDeg = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, target));
    }

    private void updateLimelight() {
        limelight.update(odometry.getPose().getHeading(), estimateDeg, PIVOT_FORWARD_IN, PIVOT_LEFT_IN);
    }

    private void readCamera() {
        updateLimelight();
        if (!limelight.sees()) {
            cameraAgrees = true;
            return;
        }
        boolean settled = clock.seconds() - lastMoveAt > CAMERA_SETTLE_S && Math.abs(estimateDeg - sentDeg) < 0.5;
        boolean quickLook = !settled && odometry.secondsSinceCellSeen() > QUICK_LOOK_IF_UNSEEN_S;
        if (!(settled || quickLook) || !odometry.isStill(CAMERA_MAX_SPEED_IN_PER_S, CAMERA_MAX_TURN_DEG_PER_S)) {
            return;
        }
        Pose robot = odometry.getPose();
        double turretInPicture = settled ? estimateDeg : angleAt(clock.seconds() - limelight.getAgeMs() / 1000.0);
        double cameraYaw = Math.toDegrees(robot.getHeading()) + TURRET_ZERO_FROM_FRONT_DEG + turretInPicture;
        double[] pivot = OdometryAim.pivotOnField(robot, PIVOT_FORWARD_IN, PIVOT_LEFT_IN);
        double[] tags = limelight.tagsMiddleOnField(cameraYaw, pivot[0], pivot[1]);
        double[] center = odometry.tagsSeen(tags[0], tags[1], limelight.getCell(), limelight.isUpCell());
        double centerTx = wrapDegrees(cameraYaw - Math.toDegrees(Math.atan2(center[1] - pivot[1], center[0] - pivot[0])));
        cameraAgrees = !settled || !odometry.isStill(STILL_SPEED_IN_PER_S, STILL_TURN_DEG_PER_S) || Math.abs(AIM_TRIM_DEG - centerTx) <= CAMERA_AGREE_DEG;
    }

    private void aim() {
        double[] point = odometry.aimPoint();
        aimSource = odometry.aimPointSource();
        if (point == null) {
            aimKnown = false;
            canReach = true;
            distanceToCell = -1;
            holdSince = -1;
            return;
        }
        double needed = angleFor(odometry.poseAhead(LAG_S), point[0], point[1]);
        canReach = Math.abs(needed) <= TURRET_LIMIT_DEG;
        double wanted = canReach ? needed : (commandDeg >= 0 ? TURRET_LIMIT_DEG : -TURRET_LIMIT_DEG);

        still = still ? odometry.isStill(MOVING_SPEED_IN_PER_S, MOVING_TURN_DEG_PER_S)
                : odometry.isStill(STILL_SPEED_IN_PER_S, STILL_TURN_DEG_PER_S);
        if (!aimKnown || !still || aimNow) {
            commandDeg = wanted;
            holdSince = -1;
            aimNow = false;
        } else if (Math.abs(wanted - commandDeg) < HOLD_DEG) {
            holdSince = -1;
        } else {
            if (holdSince < 0) {
                holdSince = clock.seconds();
            }
            if (clock.seconds() - holdSince >= HOLD_CONFIRM_S) {
                commandDeg = wanted;
                holdSince = -1;
            }
        }

        double[] pivot = OdometryAim.pivotOnField(odometry.getPose(), PIVOT_FORWARD_IN, PIVOT_LEFT_IN);
        distanceToCell = Math.hypot(point[0] - pivot[0], point[1] - pivot[1]);
        aimKnown = true;
    }

    private void search(double dt) {
        aimKnown = false;
        holdSince = -1;
        aimSource = "SEARCHING for the tags (gamepad 2 Y)";
        if (limelight.sees()) {
            searchStep = 0;
            commandDeg = sentDeg;
            return;
        }
        sweep(dt, SEARCH_SPEED_DEG_PER_S);
    }

    private void sweep(double dt, double speedDegPerSecond) {
        if (searchStep == 0) {
            searchStep = 1;
        }
        double end = searchStep == 1 ? TURRET_LIMIT_DEG : -TURRET_LIMIT_DEG;
        double stepDeg = speedDegPerSecond * dt;
        commandDeg += Math.max(-stepDeg, Math.min(stepDeg, end - commandDeg));
        if (Math.abs(end - commandDeg) < 0.5) {
            searchStep = searchStep == 1 ? 2 : 0;
        }
    }

    private void moveServo(double dt) {
        double before = sentDeg;
        double error = commandDeg - sentDeg;
        if (sentSpeed * error < 0) {
            sentSpeed = 0;
        }
        double brakeSpeed = Math.sqrt(2 * PROFILE_ACCEL_DEG_PER_S2 * Math.abs(error));
        double wantedSpeed = Math.signum(error) * Math.min(PROFILE_MAX_DEG_PER_S, brakeSpeed);
        if (Math.abs(wantedSpeed) < Math.abs(sentSpeed)) {
            sentSpeed = wantedSpeed;
        } else {
            double maxChange = PROFILE_ACCEL_DEG_PER_S2 * dt;
            sentSpeed += Math.max(-maxChange, Math.min(maxChange, wantedSpeed - sentSpeed));
        }
        double stepDeg = sentSpeed * dt;
        if (Math.abs(stepDeg) >= Math.abs(error)) {
            sentDeg = commandDeg;
            sentSpeed = 0;
        } else {
            sentDeg += stepDeg;
        }
        if (Math.abs(sentDeg - before) > 1e-6) {
            lastMoveAt = clock.seconds();
        }
        double estimateStep = SPEED_DEG_PER_S * dt;
        estimateDeg += Math.max(-estimateStep, Math.min(estimateStep, sentDeg - estimateDeg));
        angleTimes[angleNext] = clock.seconds();
        angleValues[angleNext] = estimateDeg;
        angleNext = (angleNext + 1) % angleTimes.length;
        angleCount = Math.min(angleCount + 1, angleTimes.length);
        servo.setPosition(angleToPosition(sentDeg));
    }

    private double angleAt(double time) {
        double best = sentDeg;
        for (int i = 1; i <= angleCount; i++) {
            int index = (angleNext - i + angleTimes.length) % angleTimes.length;
            best = angleValues[index];
            if (angleTimes[index] <= time) {
                break;
            }
        }
        return best;
    }

    private double angleFor(Pose robot, double x, double y) {
        double[] pivot = OdometryAim.pivotOnField(robot, PIVOT_FORWARD_IN, PIVOT_LEFT_IN);
        double bearing = Math.toDegrees(Math.atan2(y - pivot[1], x - pivot[0]));
        return wrapDegrees(bearing - Math.toDegrees(robot.getHeading()) - TURRET_ZERO_FROM_FRONT_DEG + AIM_TRIM_DEG);
    }

    public static double angleToPosition(double angleDeg) {
        double units = angleDeg / TURRET_DEGREES_PER_UNIT;
        double position = TURRET_HIGHER_IS_LEFT ? TURRET_ZERO_POS + units : TURRET_ZERO_POS - units;
        return Math.max(TURRET_MIN_POS, Math.min(TURRET_MAX_POS, position));
    }

    public static double positionToAngle(double position) {
        double units = position - TURRET_ZERO_POS;
        return (TURRET_HIGHER_IS_LEFT ? units : -units) * TURRET_DEGREES_PER_UNIT;
    }

    public boolean isAimKnown() {
        return aimKnown;
    }

    public boolean canReach() {
        return canReach;
    }

    public boolean isTurretReady() {
        return Math.abs(estimateDeg - commandDeg) <= READY_DEG;
    }

    public boolean cameraAgrees() {
        return cameraAgrees;
    }

    public boolean isAimed() {
        return aimKnown && canReach && isTurretReady() && cameraAgrees;
    }

    public double getDistanceToCell() {
        return distanceToCell;
    }

    public Alliance getAlliance() {
        return odometry.getAlliance();
    }

    public Follower getFollower() {
        return odometry.getFollower();
    }

    public LimelightAim getLimelight() {
        return limelight;
    }

    public double getFieldHeadingRad() {
        return odometry.getPose().getHeading();
    }

    public void addInitTelemetry(Telemetry telemetry) {
        telemetry.addData("alliance", "%s   (gamepad 1: X = blue, B = red)", odometry.getAlliance());
        addPosition(telemetry);
        addCamera(telemetry);
        telemetry.addLine("If the position is wrong: put the robot on the auto start spot and press gamepad 1 BACK.");
    }

    public void aimNow() {
        aimNow = true;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("AIMING WITH", aimSource);
        addCamera(telemetry);
        addPosition(telemetry);
        if (aimKnown) {
            telemetry.addData("distance to cell", "%.1f in", distanceToCell);
        }
        telemetry.addData("turret", "%.1f°%s", sentDeg, canReach ? "" : "  AT ITS LIMIT (turn the robot)");
    }

    private void addPosition(Telemetry telemetry) {
        Pose p = odometry.getPose();
        telemetry.addData("position", odometry.isPoseKnown()
                ? String.format("x %.1f  y %.1f  heading %.0f°  (%s)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()), odometry.getPositionSource())
                : "UNKNOWN (press gamepad 1 BACK on the start spot)");
    }

    private void addCamera(Telemetry telemetry) {
        if (limelight.sees()) {
            telemetry.addData("Limelight", "cell %d from %d tag(s) [%s]  %.0f in fwd, %.0f in left, %.0f in up (%.0f ms old)",
                    limelight.getCell(), limelight.getTagCount(), limelight.getIds(),
                    limelight.getForwardIn(), limelight.getLeftIn(), limelight.getUpIn(), limelight.getAgeMs());
        } else {
            telemetry.addData("Limelight", "no hive tags");
        }
    }

    public void stop() {
        limelight.stop();
    }
}
