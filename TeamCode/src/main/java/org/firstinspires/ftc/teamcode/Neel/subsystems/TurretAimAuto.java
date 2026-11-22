package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.FIELD;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.PIVOT_X;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.PIVOT_Y;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.wrapDegrees;

public final class TurretAimAuto {
    public static double TURRET_ZERO_POS = 0.5;
    public static double TURRET_ZERO_FROM_FRONT_DEG = 180;
    public static double TURRET_LIMIT_DEG = 150;
    public static double TURRET_MIN_POS = 0.25;
    public static double TURRET_MAX_POS = 0.75;
    public static double TURRET_DEGREES_PER_UNIT = 600;
    public static double TURRET_SWING_HYSTERESIS_DEG = 20;
    public static boolean TURRET_HIGHER_IS_LEFT = false;
    public static double TURRET_FORWARD_OF_CENTER_IN = 0;
    public static double TURRET_LEFT_OF_CENTER_IN = 0;
    public static double TURRET_TRIM_DEG = 0;
    public static double BALL_SPEED_IN_PER_S = 120;
    public static double TURRET_LAG_S = 0.1;
    public static double VELOCITY_SMOOTHING = 0.3;
    public static double TURRET_SPEED_DEG_PER_S = 200;
    public static double TURRET_READY_DEG = 3;
    public static boolean USE_CAMERA_AIM = true;
    public static double MAX_FRAME_AGE_MS = 300;
    public static double CAMERA_CHECK_MIN_TURN_DEG = 4;
    public static double CAMERA_CHECK_WINDOW_S = 3;
    public static double CAMERA_CHECK_MAX_SPEED_IN_PER_S = 6;
    public static double CAMERA_CHECK_MAX_ROBOT_TURN_DEG = 2;
    public static int CAMERA_CHECK_VOTES = 2;
    public static double CAMERA_UNCHECKED_GAIN = 0.3;
    public static double CAMERA_TRACK_GAIN = 0.5;
    public static double CAMERA_MAX_TURRET_RATE_DEG_PER_S = 90;
    public static double CAMERA_MAX_TURN_DEG_PER_S = 25;
    public static double CAMERA_DEADBAND_DEG = 1.0;
    public static double TURRET_HOLD_DEG = 2.0;
    public static double TURRET_STILL_SMOOTHING = 0.15;
    public static double TURRET_HOLD_CONFIRM_S = 0.4;
    public static double TURRET_HOLD_MAX_SPEED_IN_PER_S = 3;
    public static double TURRET_HOLD_MAX_TURN_DEG_PER_S = 5;
    public static double CAMERA_STILL_DEADBAND_DEG = 2.0;
    public static double TURRET_PROFILE_MAX_DEG_PER_S = 450;
    public static double TURRET_PROFILE_ACCEL_DEG_PER_S2 = 6000;
    public static double HEADING_CAMERA_MIN_TURN_DEG = 10;
    public static double HEADING_CAMERA_WINDOW_S = 3;
    public static double HEADING_CAMERA_MAX_MOVE_IN = 6;
    public static int HEADING_CAMERA_VOTES = 3;
    public static double CAMERA_SNAP_GAIN = 0.6;
    public static double CAMERA_SPOT_LIVE_S = 1.0;
    public static double CAMERA_MEMORY_S = 3.0;
    public static boolean AUTO_REACQUIRE = false;
    public static double REACQUIRE_AFTER_S = 1.0;
    public static double REACQUIRE_FOR_S = 3.0;
    public static double REACQUIRE_SWEEP_DEG = 20;
    public static double REACQUIRE_SPEED_DEG_PER_S = 40;
    public static double REACQUIRE_MAX_DISTANCE_IN = 130;
    public static int HEADING_SIGN_SETTING = 0;
    public static boolean HEADING_FROM_CAMERA = false;
    public static String HEADING_SIGN_FILE = "/sdcard/FIRST/glitch_heading_sign.txt";
    public static String TURRET_DIRECTION_FILE = "/sdcard/FIRST/glitch_turret_direction_v2.txt";
    public static double MIN_CAMERA_FLAT_IN = 10;
    public static double MAX_CAMERA_FLAT_IN = 160;
    public static double TURRET_SETTLE_S = 0.35;
    public static double TURRET_SLOWEST_DEG_PER_S = 100;
    public static double CAMERA_AGREE_DEG = 4;
    private static final int HISTORY = 256;

    private final Servo turret;
    private final Vision vision;
    private final Alliance alliance;
    private Pose lastBluePose = null;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double vx = 0;
    private double vy = 0;
    private double omegaDeg = 0;
    private int targetCell = 1;
    private double turretAngle = 0;
    private double turretCommandDeg = 0;
    private boolean turretCanReach = false;
    private boolean turretReady = false;
    private double turretCommandPos = 0.5;
    private double turretSentDeg = 0;
    private double turretSentVel = 0;
    private boolean robotStill = false;
    private double turretEstimatePos = 0.5;
    private double distanceToCell = 0;
    private boolean aimKnown = false;
    private boolean cameraAgrees = true;
    private String aimSource = "none";
    private String cameraStatus = "off";
    private double lastCameraSeenAt = -1e9;
    private double lastFrameCapture = -1e9;
    private double checkRefRot = Double.NaN;
    private double checkRefHeading = 0;
    private double gyroTurnDeg = 0;
    private double lastCameraXSign = Vision.CAMERA_X_SIGN;
    private double steadyCommandDeg = 0;
    private double commandChangedAt = -1e9;
    private double lastSpotUpdateAt = -1e9;
    private double heldCommandDeg = 0;
    private double holdExceededSince = 0;
    private double smoothCommandDeg = 0;
    private double signRefTime = -1e9;
    private double signRefBearing = 0;
    private double signRefHeading = 0;
    private double signRefX = 0;
    private double signRefY = 0;
    private int signRefCell = -1;
    private int headingCameraVotes = 0;
    private boolean mirrorConfirmed = false;
    private double checkRefAngle = 0;
    private double checkRefTime = 0;
    private int checkRefCell = -1;
    private double checkLastCapture = -1e9;
    private int cameraDirectionVotes = 0;
    private String cameraFixMessage = "";
    private final double defaultRise = FieldMap.UP_TAGS_HEIGHT_IN - Vision.CAMERA_HEIGHT_IN;
    private double reacquireOffsetDeg = 0;
    private double reacquireDirection = 1;
    private boolean shootingNow = false;
    private double headingSign = 1;
    private double anchorRaw = 0;
    private double anchorTrue = 0;
    private double lastRawHeading = Double.NaN;
    private boolean poseWasSet = false;
    private double fixedX = 0;
    private double fixedY = 0;
    private double lastRawX = 0;
    private double lastRawY = 0;
    private double lastRawH = 0;
    private final double[][] cameraSpot = new double[2][3];
    private final boolean[] haveCameraSpot = new boolean[2];
    private final double[] cameraSpotDistX = new double[2];
    private final double[] cameraSpotDistY = new double[2];
    private int lastCameraCell = -1;
    private final double[][] history = new double[HISTORY][5];
    private int historyCount = 0;
    private int historyNext = 0;
    private final ElapsedTime clock = new ElapsedTime();
    private double turretLastMovingAt = 0;
    private double turretSlowEstimatePos = 0.5;

    public TurretAimAuto(HardwareMap hardwareMap, Alliance alliance, Pose startPose) {
        this.alliance = alliance;
        loadHeadingSign();
        loadTurretDirection();
        turret = hardwareMap.get(Servo.class, "turret");
        resetFixedPose(startPose);
        vision = USE_CAMERA_AIM ? new Vision(hardwareMap, alliance) : null;
        turretLastMovingAt = clock.seconds() + TURRET_LIMIT_DEG / TURRET_SLOWEST_DEG_PER_S;
        turretSentDeg = TURRET_ZERO_POS * TURRET_DEGREES_PER_UNIT;
    }

    public void start() {
        loopTimer.reset();
        lastCameraSeenAt = clock.seconds();
    }

    public Vision getVision() {
        return vision;
    }

    public void stop() {
        if (vision != null) {
            vision.stop();
        }
    }

    public boolean isAimKnown() {
        return aimKnown;
    }

    public boolean isAimed() {
        return aimKnown && turretCanReach && turretReady && cameraAgrees;
    }

    public double getDistanceToCell() {
        return distanceToCell;
    }

    public void updateFromPoseAuto(Pose pose, int cell, boolean shooting) {
        if (!isGood(pose)) {
            aimKnown = false;
            return;
        }
        updateFixedPoseAuto(pose);
        double dt = updateVelocityAuto();
        noteHeadingAuto(pose.getHeading());
        shootingNow = shooting;
        aimTurretAuto(dt, cell);
    }

    public void addTelemetryAuto(Telemetry telemetry) {
        telemetry.addData("alliance", alliance);
        if (headingSign < 0) {
            telemetry.addLine("Pinpoint heading correction active for turret aiming.");
        }
        telemetry.addData("AIMING WITH", aimSource);
        telemetry.addData("camera", cameraStatus);
        if (!cameraFixMessage.isEmpty()) {
            telemetry.addLine(cameraFixMessage);
        }
        if (!Vision.getCameraFixMessage().isEmpty()) {
            telemetry.addLine(Vision.getCameraFixMessage());
        }
        Pose pose = currentPoseAuto();
        telemetry.addData("position", "OpMode Pinpoint pose");
        telemetry.addData("robot", "x %.1f  y %.1f  h %.1f°",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("aiming at", (blueFrameCellIsRear() ? "REAR" : "AUDIENCE") + " cell");
        telemetry.addData("distance", "%.1f in", distanceToCell);
        telemetry.addData("robot speed", "%.0f in/s", Math.hypot(vx, vy));
        telemetry.addData("turret angle", "%.1f° %s", turretAngle,
                !aimKnown ? "(searching for the tags)"
                        : !turretCanReach ? "(AT LIMIT: can't aim yet - turn the robot a bit, or it will swing around)"
                        : !turretReady ? "moving" : !cameraAgrees ? "camera says still turning" : "ready");
    }

    private double profiledPosition(double dt) {
        double goalDeg = turretCommandPos * TURRET_DEGREES_PER_UNIT;
        double err = goalDeg - turretSentDeg;
        if (dt <= 0 || dt > 0.25) {
            turretSentDeg = goalDeg;
            turretSentVel = 0;
            return turretCommandPos;
        }
        if (turretSentVel * err < 0) {
            turretSentVel = 0;
        }
        double brakeSpeed = Math.sqrt(2 * TURRET_PROFILE_ACCEL_DEG_PER_S2 * Math.abs(err));
        double wanted = Math.signum(err) * Math.min(TURRET_PROFILE_MAX_DEG_PER_S, brakeSpeed);
        if (Math.abs(wanted) < Math.abs(turretSentVel)) {
            turretSentVel = wanted;
        } else {
            double maxChange = TURRET_PROFILE_ACCEL_DEG_PER_S2 * dt;
            turretSentVel += Math.max(-maxChange, Math.min(maxChange, wanted - turretSentVel));
        }
        double step = turretSentVel * dt;
        if (Math.abs(step) >= Math.abs(err)) {
            turretSentDeg = goalDeg;
            turretSentVel = 0;
        } else {
            turretSentDeg += step;
        }
        return turretSentDeg / TURRET_DEGREES_PER_UNIT;
    }

    private void commandTurret(double position, double dt) {
        if (Double.isNaN(position) || Double.isInfinite(position)) {
            position = turretCommandPos;
        }
        turretCommandPos = Math.max(TURRET_MIN_POS, Math.min(TURRET_MAX_POS, position));
        double sentPos = profiledPosition(dt);
        turret.setPosition(sentPos);
        double commandDeg = turretCommandPos * TURRET_DEGREES_PER_UNIT;
        if (dt > 0) {
            double rate = Math.abs(commandDeg - steadyCommandDeg) / dt;
            if (rate > CAMERA_MAX_TURRET_RATE_DEG_PER_S) {
                commandChangedAt = clock.seconds();
            }
        }
        steadyCommandDeg = commandDeg;
        double maxStep = TURRET_SPEED_DEG_PER_S / TURRET_DEGREES_PER_UNIT * dt;
        double gap = sentPos - turretEstimatePos;
        turretEstimatePos += Math.max(-maxStep, Math.min(maxStep, gap));
        turretReady = Math.abs(turretCommandPos - turretEstimatePos) * TURRET_DEGREES_PER_UNIT <= TURRET_READY_DEG;

        double slowStep = TURRET_SLOWEST_DEG_PER_S / TURRET_DEGREES_PER_UNIT * dt;
        double slowGap = sentPos - turretSlowEstimatePos;
        turretSlowEstimatePos += Math.max(-slowStep, Math.min(slowStep, slowGap));
        if (!turretReady || Math.abs(turretCommandPos - turretSlowEstimatePos) * TURRET_DEGREES_PER_UNIT > TURRET_READY_DEG) {
            turretLastMovingAt = clock.seconds();
        }
    }

    private double turretActualDeg() {
        return positionToAngle(turretEstimatePos);
    }

    public static double positionToAngle(double position) {
        double units = position - TURRET_ZERO_POS;
        return (TURRET_HIGHER_IS_LEFT ? units : -units) * TURRET_DEGREES_PER_UNIT;
    }

    public static double angleToPosition(double angleDeg) {
        double units = angleDeg / TURRET_DEGREES_PER_UNIT;
        return TURRET_HIGHER_IS_LEFT ? TURRET_ZERO_POS + units : TURRET_ZERO_POS - units;
    }

    private boolean turretSettledSince(double t) {
        return turretLastMovingAt <= t - TURRET_SETTLE_S && commandChangedAt <= t - TURRET_SETTLE_S;
    }

    private double rowY(int cell) {
        double fromPivot = FieldMap.UP_TAGS_FROM_PIVOT_IN;
        return cell == 1 ? PIVOT_Y + fromPivot : PIVOT_Y - fromPivot;
    }

    private double guessFlatDistance(double fromX, double fromY, int cell) {
        if (cell >= 0 && haveCameraSpot[cell]) {
            return Math.hypot(cameraSpot[cell][0] - fromX, cameraSpot[cell][1] - fromY);
        }
        double mapY = targetCell == 1 ? PIVOT_Y + FieldMap.CELL_AIM_FROM_PIVOT_IN : PIVOT_Y - FieldMap.CELL_AIM_FROM_PIVOT_IN;
        return Math.hypot(PIVOT_X - fromX, mapY - fromY);
    }

    private void remember(double t, double x, double y, double headingDeg, double turretDeg) {
        double[] row = history[historyNext];
        row[0] = t;
        row[1] = x;
        row[2] = y;
        row[3] = headingDeg;
        row[4] = turretDeg;
        historyNext = (historyNext + 1) % HISTORY;
        historyCount = Math.min(historyCount + 1, HISTORY);
    }

    private double[] poseAt(double t) {
        int newest = (historyNext - 1 + HISTORY) % HISTORY;
        double[] after = history[newest];
        if (historyCount <= 1 || t >= after[0]) {
            return new double[]{after[1], after[2], after[3], after[4]};
        }
        for (int i = 1; i < historyCount; i++) {
            double[] before = history[(newest - i + HISTORY) % HISTORY];
            if (before[0] <= t) {
                double span = after[0] - before[0];
                double f = span > 1e-9 ? (t - before[0]) / span : 1;
                return new double[]{
                        before[1] + f * (after[1] - before[1]),
                        before[2] + f * (after[2] - before[2]),
                        before[3] + f * wrapDegrees(after[3] - before[3]),
                        before[4] + f * (after[4] - before[4])
                };
            }
            after = before;
        }
        return new double[]{after[1], after[2], after[3], after[4]};
    }

    private boolean checkCameraDirection(double now) {
        if (!vision.hasFreshTag() || vision.getFrameAgeMs() > MAX_FRAME_AGE_MS) {
            return false;
        }
        if (Vision.CAMERA_X_SIGN != lastCameraXSign) {
            lastCameraXSign = Vision.CAMERA_X_SIGN;
            cameraDirectionVotes = 0;
            mirrorConfirmed = false;
            checkRefRot = Double.NaN;
            signRefTime = -1e9;
            haveCameraSpot[0] = false;
            haveCameraSpot[1] = false;
            lastCameraCell = -1;
        }
        double capture = now - vision.getFrameAgeMs() / 1000.0;
        if (capture <= checkLastCapture + 0.002) {
            return false;
        }
        checkLastCapture = capture;
        double[] then = poseAt(capture);
        double rot = then[3];
        double angle = vision.getAngle();
        int cell = vision.getTargetCluster();
        if (!turretSettledSince(capture)) {
            return false;
        }
        boolean slow = Math.hypot(vx, vy) <= CAMERA_CHECK_MAX_SPEED_IN_PER_S;
        if (Double.isNaN(checkRefRot) || !slow || cell != checkRefCell
                || capture - checkRefTime > CAMERA_CHECK_WINDOW_S
                || Math.abs(gyroTurnDeg - checkRefHeading) > CAMERA_CHECK_MAX_ROBOT_TURN_DEG) {
            checkRefRot = slow ? rot : Double.NaN;
            checkRefHeading = gyroTurnDeg;
            checkRefAngle = angle;
            checkRefTime = capture;
            checkRefCell = cell;
            return false;
        }
        double turned = wrapDegrees(rot - checkRefRot);
        if (Math.abs(turned) < CAMERA_CHECK_MIN_TURN_DEG) {
            return false;
        }
        double moved = wrapDegrees(angle - checkRefAngle);
        checkRefRot = rot;
        checkRefHeading = gyroTurnDeg;
        checkRefAngle = angle;
        checkRefTime = capture;
        double ratio = Math.abs(moved) / Math.abs(turned);
        if (ratio < 0.6 || ratio > 1.6) {
            return false;
        }
        cameraDirectionVotes += moved * turned > 0 ? 1 : -1;
        cameraDirectionVotes = Math.max(-CAMERA_CHECK_VOTES, Math.min(CAMERA_CHECK_VOTES, cameraDirectionVotes));
        if (cameraDirectionVotes >= CAMERA_CHECK_VOTES && !mirrorConfirmed) {
            mirrorConfirmed = true;
            saveTurretDirection();
        }
        if (mirrorConfirmed || cameraDirectionVotes > -CAMERA_CHECK_VOTES) {
            return false;
        }
        TURRET_HIGHER_IS_LEFT = !TURRET_HIGHER_IS_LEFT;
        saveTurretDirection();
        turretCommandDeg = turretActualDeg();
        historyCount = 0;
        lastBluePose = null;
        cameraDirectionVotes = 0;
        mirrorConfirmed = false;
        signRefTime = -1e9;
        checkRefRot = Double.NaN;
        haveCameraSpot[0] = false;
        haveCameraSpot[1] = false;
        lastCameraCell = -1;
        cameraFixMessage = "turret direction was backwards: fixed automatically";
        return true;
    }

    public static void loadTurretDirection() {
        try (java.io.BufferedReader reader = new java.io.BufferedReader(new java.io.FileReader(new java.io.File(TURRET_DIRECTION_FILE)))) {
            TURRET_HIGHER_IS_LEFT = Integer.parseInt(reader.readLine().trim()) > 0;
        } catch (Exception ignored) {
        }
    }

    private void saveTurretDirection() {
        try (java.io.FileWriter writer = new java.io.FileWriter(new java.io.File(TURRET_DIRECTION_FILE), false)) {
            writer.write(TURRET_HIGHER_IS_LEFT ? "1" : "-1");
        } catch (Exception ignored) {
        }
    }

    private boolean checkHeadingFromCamera(double capture, double[] then, int cell) {
        if (!HEADING_FROM_CAMERA || !mirrorConfirmed || HEADING_SIGN_SETTING != 0) {
            return false;
        }
        double bearing = then[2] + TURRET_ZERO_FROM_FRONT_DEG + then[3] - vision.getAngle();
        boolean fresh = capture - signRefTime <= HEADING_CAMERA_WINDOW_S && cell == signRefCell
                && Math.hypot(then[0] - signRefX, then[1] - signRefY) <= HEADING_CAMERA_MAX_MOVE_IN;
        if (!fresh) {
            setSignRef(capture, bearing, then, cell);
            return false;
        }
        double turned = wrapDegrees(then[2] - signRefHeading);
        if (Math.abs(turned) < HEADING_CAMERA_MIN_TURN_DEG) {
            return false;
        }
        double drift = wrapDegrees(bearing - signRefBearing);
        double driftIfFlipped = wrapDegrees(drift - 2 * turned);
        setSignRef(capture, bearing, then, cell);
        if (Math.abs(driftIfFlipped) < 0.5 * Math.abs(turned) && Math.abs(drift) > 1.5 * Math.abs(turned)) {
            headingCameraVotes--;
        } else if (Math.abs(drift) < 0.5 * Math.abs(turned)) {
            headingCameraVotes = Math.min(HEADING_CAMERA_VOTES, headingCameraVotes + 1);
        }
        if (headingCameraVotes <= -HEADING_CAMERA_VOTES) {
            headingCameraVotes = 0;
            signRefTime = -1e9;
            setHeadingSign(-headingSign);
            haveCameraSpot[0] = false;
            haveCameraSpot[1] = false;
            lastCameraCell = -1;
            cameraFixMessage = "Pinpoint heading was backwards: fixed automatically";
            return true;
        }
        return false;
    }

    private void setSignRef(double capture, double bearing, double[] then, int cell) {
        signRefTime = capture;
        signRefBearing = bearing;
        signRefHeading = then[2];
        signRefX = then[0];
        signRefY = then[1];
        signRefCell = cell;
    }

    private boolean blueFrameCellIsRear() {
        return alliance == Alliance.RED ? targetCell == 0 : targetCell == 1;
    }

    private void resetFixedPose(Pose pose) {
        anchorRaw = pose.getHeading();
        anchorTrue = pose.getHeading();
        fixedX = pose.getX();
        fixedY = pose.getY();
        lastRawX = pose.getX();
        lastRawY = pose.getY();
        lastRawH = pose.getHeading();
        poseWasSet = true;
    }

    private static double wrapDegreesRad(double radians) {
        return Math.toRadians(wrapDegrees(Math.toDegrees(radians)));
    }

    private static boolean isGood(Pose pose) {
        return pose != null && !Double.isNaN(pose.getX()) && !Double.isNaN(pose.getY()) && !Double.isNaN(pose.getHeading())
                && !Double.isInfinite(pose.getX()) && !Double.isInfinite(pose.getY()) && !Double.isInfinite(pose.getHeading());
    }

    private void setHeadingSign(double sign) {
        if (sign == headingSign) {
            return;
        }
        headingSign = sign;
        lastBluePose = null;
        historyCount = 0;
        try (java.io.FileWriter writer = new java.io.FileWriter(new java.io.File(HEADING_SIGN_FILE), false)) {
            writer.write(sign > 0 ? "1" : "-1");
        } catch (Exception ignored) {
        }
    }

    private void loadHeadingSign() {
        if (HEADING_SIGN_SETTING != 0) {
            headingSign = HEADING_SIGN_SETTING > 0 ? 1 : -1;
            return;
        }
        try (java.io.BufferedReader reader = new java.io.BufferedReader(new java.io.FileReader(new java.io.File(HEADING_SIGN_FILE)))) {
            headingSign = Integer.parseInt(reader.readLine().trim()) < 0 ? -1 : 1;
        } catch (Exception e) {
            headingSign = 1;
        }
    }

    private Pose toBlueFrame(Pose real) {
        if (alliance == Alliance.RED) {
            return new Pose(FIELD - real.getX(), FIELD - real.getY(), real.getHeading() - Math.PI);
        }
        return real;
    }

    private void updateFixedPoseAuto(Pose raw) {
        if (!isGood(raw)) {
            return;
        }
        double dx = raw.getX() - lastRawX;
        double dy = raw.getY() - lastRawY;
        if (Math.hypot(dx, dy) > 12) {
            dx = 0;
            dy = 0;
        }
        double rawMid = lastRawH + 0.5 * wrapDegreesRad(raw.getHeading() - lastRawH);
        double turn = (anchorTrue + headingSign * (rawMid - anchorRaw)) - rawMid;
        fixedX += dx * Math.cos(turn) - dy * Math.sin(turn);
        fixedY += dx * Math.sin(turn) + dy * Math.cos(turn);
        lastRawX = raw.getX();
        lastRawY = raw.getY();
        lastRawH = raw.getHeading();
    }

    private Pose currentPoseAuto() {
        return new Pose(fixedX, fixedY, anchorTrue + headingSign * (lastRawH - anchorRaw));
    }

    private double updateVelocityAuto() {
        Pose now = toBlueFrame(currentPoseAuto());
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();
        if (lastBluePose != null) {
            double a = VELOCITY_SMOOTHING;
            vx += a * ((now.getX() - lastBluePose.getX()) / dt - vx);
            vy += a * ((now.getY() - lastBluePose.getY()) / dt - vy);
            omegaDeg += a * (wrapDegrees(Math.toDegrees(now.getHeading() - lastBluePose.getHeading())) / dt - omegaDeg);
        }
        lastBluePose = now;
        return dt;
    }

    private void noteHeadingAuto(double raw) {
        if (poseWasSet || Double.isNaN(lastRawHeading)) {
            poseWasSet = false;
        } else {
            gyroTurnDeg += wrapDegrees(Math.toDegrees(raw - lastRawHeading));
        }
        lastRawHeading = raw;
    }

    private void aimTurretAuto(double dt, int cell) {
        Pose p = toBlueFrame(currentPoseAuto());
        double headingDeg = Math.toDegrees(p.getHeading());
        double now = clock.seconds();
        remember(now, p.getX(), p.getY(), headingDeg, turretActualDeg());
        robotStill = Math.hypot(vx, vy) < TURRET_HOLD_MAX_SPEED_IN_PER_S && Math.abs(omegaDeg) < TURRET_HOLD_MAX_TURN_DEG_PER_S;
        targetCell = cell;

        boolean cameraFresh = false;
        if (vision != null) {
            int wantedCell = targetCell;
            vision.update(headingDeg + TURRET_ZERO_FROM_FRONT_DEG + turretActualDeg(), wantedCell);
            if (checkCameraDirection(now)) {
                vision.resetTracking();
                vision.update(headingDeg + TURRET_ZERO_FROM_FRONT_DEG + turretActualDeg(), wantedCell);
            }
            cameraFresh = vision.hasFreshTag() && vision.getFrameAgeMs() <= MAX_FRAME_AGE_MS;
        }

        double aimX;
        double aimY;
        double distX;
        double distY;
        double rise;
        boolean addTrim = false;
        double captureTime = cameraFresh ? now - vision.getFrameAgeMs() / 1000.0 : now;
        boolean cameraTrusted = cameraFresh && turretSettledSince(captureTime) && Math.abs(omegaDeg) <= CAMERA_MAX_TURN_DEG_PER_S;
        if (vision == null) {
            cameraStatus = "off (USE_CAMERA_AIM is false)";
        } else if (vision.getRawTagIds().isEmpty()) {
            cameraStatus = "no tags in view";
        } else if (!vision.sawOurAllianceTag()) {
            cameraStatus = "sees tags " + vision.getRawTagIds() + " but none are " + vision.getAlliance()
                    + " hive tags";
        } else if (!cameraFresh) {
            cameraStatus = String.format("sees tags %s but the picture is %.0f ms old (limit %.0f)",
                    vision.getRawTagIds(), vision.getFrameAgeMs(), MAX_FRAME_AGE_MS);
        } else if (!cameraTrusted) {
            cameraStatus = "sees tags " + vision.getRawTagIds() + ", holding aim until the turret settles";
        } else {
            cameraStatus = String.format("sees tags %s, tracking (%.0f ms old)", vision.getRawTagIds(), vision.getFrameAgeMs());
        }
        int seenCell = cameraFresh ? vision.getTargetCluster() : -1;
        boolean got3d = cameraFresh && vision.hasFresh3d() && vision.getFlatDistance() >= MIN_CAMERA_FLAT_IN
                && vision.getFlatDistance() <= MAX_CAMERA_FLAT_IN;
        double secondsSinceSeen = now - lastCameraSeenAt;
        if (cameraFresh) {
            lastCameraSeenAt = now;
        }

        boolean validCell = seenCell == 0 || seenCell == 1;
        boolean newFrame = cameraFresh && captureTime > lastFrameCapture + 0.002;
        if (newFrame) {
            lastFrameCapture = captureTime;
        }
        boolean usable = newFrame && cameraTrusted && validCell;
        boolean spotLive = validCell && haveCameraSpot[seenCell] && seenCell == lastCameraCell
                && now - lastSpotUpdateAt <= CAMERA_SPOT_LIVE_S;
        if (cameraFresh && (usable || spotLive)) {
            double[] then = poseAt(captureTime);
            if (usable && checkHeadingFromCamera(captureTime, then, seenCell)) {
                commandTurret(turretCommandPos, dt);
                return;
            }
            double thenH = Math.toRadians(then[2]);
            double bearing = Math.toRadians(then[2] + TURRET_ZERO_FROM_FRONT_DEG + then[3] - vision.getAngle());
            double thenTurretX = then[0] + TURRET_FORWARD_OF_CENTER_IN * Math.cos(thenH) - TURRET_LEFT_OF_CENTER_IN * Math.sin(thenH);
            double thenTurretY = then[1] + TURRET_FORWARD_OF_CENTER_IN * Math.sin(thenH) + TURRET_LEFT_OF_CENTER_IN * Math.cos(thenH);
            double flat;
            double seenRise;
            if (got3d) {
                flat = vision.hasMeasuredScale() ? vision.getScaledFlatDistance() : vision.getFlatDistance();
                seenRise = vision.hasMeasuredScale() ? vision.getScaledRise() : vision.getRise();
                aimSource = "LIMELIGHT (" + vision.getTagsUsed() + " tag" + (vision.getTagsUsed() == 1 ? "" : "s") + ")";
            } else {
                flat = guessFlatDistance(thenTurretX, thenTurretY, seenCell);
                seenRise = haveCameraSpot[seenCell] ? cameraSpot[seenCell][2] : defaultRise;
                aimSource = "LIMELIGHT (no 3D: turn on Full 3D)";
            }
            if (usable) {
                double seenDistX = thenTurretX + flat * Math.cos(bearing);
                double seenDistY = thenTurretY + flat * Math.sin(bearing);
                double seenAimX = seenDistX;
                double seenAimY = seenDistY;
                double deeper = FieldMap.CELL_AIM_FROM_PIVOT_IN
                        - FieldMap.UP_TAGS_FROM_PIVOT_IN;
                    seenAimY += seenCell == 1 ? deeper : -deeper;
                double gain = mirrorConfirmed ? CAMERA_TRACK_GAIN : Math.min(CAMERA_TRACK_GAIN, CAMERA_UNCHECKED_GAIN);
                if (!spotLive) {
                    double look = Math.toRadians(then[2] + TURRET_ZERO_FROM_FRONT_DEG + then[3]);
                    double lookDistX = thenTurretX + flat * Math.cos(look);
                    double lookDistY = thenTurretY + flat * Math.sin(look);
                    cameraSpot[seenCell][0] = lookDistX + (seenAimX - seenDistX);
                    cameraSpot[seenCell][1] = lookDistY + (seenAimY - seenDistY);
                    cameraSpot[seenCell][2] = seenRise;
                    cameraSpotDistX[seenCell] = lookDistX;
                    cameraSpotDistY[seenCell] = lookDistY;
                    gain = mirrorConfirmed ? CAMERA_SNAP_GAIN : Math.min(CAMERA_SNAP_GAIN, CAMERA_UNCHECKED_GAIN);
                } else {
                    double seenBearing = Math.atan2(seenAimY - thenTurretY, seenAimX - thenTurretX);
                    double spotBearing = Math.atan2(cameraSpot[seenCell][1] - thenTurretY, cameraSpot[seenCell][0] - thenTurretX);
                    if (Math.abs(wrapDegrees(Math.toDegrees(seenBearing - spotBearing))) < (robotStill ? CAMERA_STILL_DEADBAND_DEG : CAMERA_DEADBAND_DEG)) {
                        gain = 0;
                    }
                }
                cameraSpot[seenCell][0] += gain * (seenAimX - cameraSpot[seenCell][0]);
                cameraSpot[seenCell][1] += gain * (seenAimY - cameraSpot[seenCell][1]);
                cameraSpot[seenCell][2] += gain * (seenRise - cameraSpot[seenCell][2]);
                cameraSpotDistX[seenCell] += gain * (seenDistX - cameraSpotDistX[seenCell]);
                cameraSpotDistY[seenCell] += gain * (seenDistY - cameraSpotDistY[seenCell]);
                haveCameraSpot[seenCell] = true;
                lastCameraCell = seenCell;
                lastSpotUpdateAt = now;
            }
            targetCell = seenCell;
            aimX = cameraSpot[seenCell][0];
            aimY = cameraSpot[seenCell][1];
            rise = cameraSpot[seenCell][2];
            distX = cameraSpotDistX[seenCell];
            distY = cameraSpotDistY[seenCell];
        } else if (haveCameraSpot[targetCell] && targetCell == lastCameraCell && secondsSinceSeen < CAMERA_MEMORY_S) {
            double[] spot = cameraSpot[targetCell];
            aimX = spot[0];
            aimY = spot[1];
            distX = cameraSpotDistX[targetCell];
            distY = cameraSpotDistY[targetCell];
            rise = spot[2];
            aimSource = "ODOMETRY (holding where the Limelight last saw the cell)";
        } else {
            aimX = PIVOT_X;
            aimY = targetCell == 1 ? PIVOT_Y + FieldMap.CELL_AIM_FROM_PIVOT_IN : PIVOT_Y - FieldMap.CELL_AIM_FROM_PIVOT_IN;
            distX = PIVOT_X;
            distY = rowY(targetCell);
            rise = defaultRise;
            addTrim = true;
            aimSource = cameraFresh ? "ODOMETRY (camera sees tags, waiting for a steady view)" : "ODOMETRY (camera can't see the tags)";
        }
        aimKnown = true;

        cameraAgrees = true;
        if (cameraFresh) {
            double[] then = poseAt(captureTime);
            double thenH = Math.toRadians(then[2]);
            double thenTurretX = then[0] + TURRET_FORWARD_OF_CENTER_IN * Math.cos(thenH) - TURRET_LEFT_OF_CENTER_IN * Math.sin(thenH);
            double thenTurretY = then[1] + TURRET_FORWARD_OF_CENTER_IN * Math.sin(thenH) + TURRET_LEFT_OF_CENTER_IN * Math.cos(thenH);
            double aimBearing = Math.toDegrees(Math.atan2(distY - thenTurretY, distX - thenTurretX));
            double expectedAngle = wrapDegrees(then[2] + TURRET_ZERO_FROM_FRONT_DEG + then[3] - aimBearing);
            cameraAgrees = Math.abs(wrapDegrees(vision.getAngle() - expectedAngle)) <= CAMERA_AGREE_DEG;
        }

        double leadVx = robotStill ? 0 : vx;
        double leadVy = robotStill ? 0 : vy;
        double leadOmega = robotStill ? 0 : omegaDeg;
        double x = p.getX() + leadVx * TURRET_LAG_S;
        double y = p.getY() + leadVy * TURRET_LAG_S;
        double h = p.getHeading() + Math.toRadians(leadOmega * TURRET_LAG_S);
        double turretX = x + TURRET_FORWARD_OF_CENTER_IN * Math.cos(h) - TURRET_LEFT_OF_CENTER_IN * Math.sin(h);
        double turretY = y + TURRET_FORWARD_OF_CENTER_IN * Math.sin(h) + TURRET_LEFT_OF_CENTER_IN * Math.cos(h);

        double flightTime = 0;
        double leadX = aimX;
        double leadY = aimY;
        for (int i = 0; i < 5; i++) {
            leadX = aimX - leadVx * flightTime;
            leadY = aimY - leadVy * flightTime;
            flightTime = Math.hypot(leadX - turretX, leadY - turretY) / BALL_SPEED_IN_PER_S;
        }

        double bearing = Math.toDegrees(Math.atan2(leadY - turretY, leadX - turretX));
        turretAngle = wrapDegrees(bearing - Math.toDegrees(h) - TURRET_ZERO_FROM_FRONT_DEG + (addTrim ? TURRET_TRIM_DEG : 0));

        double nearest = turretAngle;
        if (Math.abs(turretAngle + 360 - turretCommandDeg) < Math.abs(nearest - turretCommandDeg)) {
            nearest = turretAngle + 360;
        }
        if (Math.abs(turretAngle - 360 - turretCommandDeg) < Math.abs(nearest - turretCommandDeg)) {
            nearest = turretAngle - 360;
        }
        boolean otherWayInRange = Math.abs(turretAngle) <= TURRET_LIMIT_DEG;
        if (Math.abs(nearest) <= TURRET_LIMIT_DEG) {
            turretCanReach = true;
            turretCommandDeg = nearest;
        } else if (otherWayInRange && Math.abs(nearest) > TURRET_LIMIT_DEG + TURRET_SWING_HYSTERESIS_DEG) {
            turretCanReach = true;
            turretCommandDeg = turretAngle;
        } else {
            turretCanReach = false;
            turretCommandDeg = nearest > 0 ? TURRET_LIMIT_DEG : -TURRET_LIMIT_DEG;
        }

        double flat = Math.hypot(distX - leadVx * flightTime - x, distY - leadVy * flightTime - y);
        distanceToCell = Math.hypot(flat, rise);

        boolean reacquire = AUTO_REACQUIRE && vision != null && !cameraFresh && !shootingNow && turretCanReach
                && now - lastCameraSeenAt > REACQUIRE_AFTER_S && now - lastCameraSeenAt < REACQUIRE_AFTER_S + REACQUIRE_FOR_S
                && flat <= REACQUIRE_MAX_DISTANCE_IN;
        if (reacquire) {
            reacquireOffsetDeg += reacquireDirection * REACQUIRE_SPEED_DEG_PER_S * dt;
            if (reacquireOffsetDeg >= REACQUIRE_SWEEP_DEG) {
                reacquireOffsetDeg = REACQUIRE_SWEEP_DEG;
                reacquireDirection = -1;
            } else if (reacquireOffsetDeg <= -REACQUIRE_SWEEP_DEG) {
                reacquireOffsetDeg = -REACQUIRE_SWEEP_DEG;
                reacquireDirection = 1;
            }
            aimSource = "ODOMETRY (camera lost the tags: looking around to find them)";
        } else {
            reacquireOffsetDeg = 0;
        }
        double command = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, turretCommandDeg + reacquireOffsetDeg));
        if (robotStill && !reacquire) {
            smoothCommandDeg += TURRET_STILL_SMOOTHING * (command - smoothCommandDeg);
            command = smoothCommandDeg;
            if (Math.abs(command - heldCommandDeg) < TURRET_HOLD_DEG) {
                holdExceededSince = now;
                command = heldCommandDeg;
            } else if (now - holdExceededSince < TURRET_HOLD_CONFIRM_S) {
                command = heldCommandDeg;
            }
        } else {
            holdExceededSince = now;
            smoothCommandDeg = command;
        }
        heldCommandDeg = command;
        commandTurret(angleToPosition(command), dt);
    }
}
