package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.FIELD;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.PIVOT_X;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.PIVOT_Y;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.wrapDegrees;

public class TurretAim {
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
    public static double AUTO_POSE_MAX_AGE_S = 120;
    public static double SAVED_FILE_MAX_AGE_S = 120;
    public static double KEPT_MATCH_IN = 24;
    public static double KEPT_MATCH_DEG = 45;
    public static double TURRET_SPEED_DEG_PER_S = 200;
    public static double TURRET_READY_DEG = 3;

    public static boolean USE_TAG_LOCATING = true;
    public static double CAMERA_ON_TURRET_FORWARD_IN = 0;
    public static double CAMERA_ON_TURRET_LEFT_IN = -Vision.CAMERA_RIGHT_OF_CENTER_IN;
    public static double RELOCATE_MAX_SPEED_IN_S = 8;
    public static double RELOCATE_MAX_TURN_DEG_S = 30;

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
    public static double SEARCH_SPEED_DEG_PER_S = 60;
    public static double SEARCH_LIMIT_DEG = 170;
    public static double HOLD_FOR_CAMERA_S = 2.0;
    public static double CELL_SWITCH_HYSTERESIS_IN = 4;
    public static double POSE_CORRECTION_GAIN = 0.5;
    public static double POSE_CORRECTION_PERIOD_S = 0.05;
    public static double POSE_CORRECTION_MAX_STEP_IN = 3;
    public static double POSE_CORRECTION_MIN_STEP_IN = 0.05;
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
    public static double HEADING_CHECK_STICK = 0.4;
    public static double HEADING_CHECK_MIN_DEG_PER_S = 30;
    public static int HEADING_CHECK_VOTES = 12;
    public static double MIN_CAMERA_FLAT_IN = 10;
    public static double MAX_CAMERA_FLAT_IN = 160;
    public static double MAX_CAMERA_CORRECTION_IN = 30;
    public static int BIG_ERROR_FRAMES = 10;
    public static double BIG_ERROR_AGREE_IN = 6;
    public static double BIG_ERROR_MAX_IN = 100;
    public static double TURRET_SETTLE_S = 0.35;
    public static double TURRET_SLOWEST_DEG_PER_S = 100;
    public static double CAMERA_AGREE_DEG = 4;
    public static double RELOCATE_TURRET_STEADY_DEG = 0.5;
    public static double RELOCATE_FRAME_MAX_AGE_S = 0.15;
    public static double RELOCATE_SETTLE_S = 0.1;
    public static double ROBOT_HALF_IN = 9;

    private final HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Follower follower;
    private Servo turret;
    private TagLocalizer localizer;
    private Vision vision;
    private Alliance alliance = Alliance.BLUE;

    private boolean located = false;
    private String positionSource = "none";
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
    private double searchDirection = 1;
    private double lastCameraSeenAt = -1e9;
    private double lastCorrectionAt = -1e9;
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
    private boolean manualSearch = false;
    private int manualSearchEnds = 0;
    private double smoothCommandDeg = 0;
    private boolean searchButtonWas = false;
    private boolean holdUntilCamera = false;
    private double holdUntilCameraSince = 0;
    private double signRefTime = -1e9;
    private double signRefBearing = 0;
    private double signRefHeading = 0;
    private double signRefX = 0;
    private double signRefY = 0;
    private int signRefCell = -1;
    private int headingCameraVotes = 0;
    private boolean mirrorConfirmed = false;
    private boolean headingFromCamera = false;
    private double checkRefAngle = 0;
    private double checkRefTime = 0;
    private int checkRefCell = -1;
    private int checkRefTags = -1;
    private double checkLastCapture = -1e9;
    private int cameraDirectionVotes = 0;
    private String cameraFixMessage = "";
    private double lastCameraErrorIn = 0;
    private int bigErrorCount = 0;
    private double bigErrorX = 0;
    private double bigErrorY = 0;
    private double lastRise = FieldMap.UP_TAGS_HEIGHT_IN - Vision.CAMERA_HEIGHT_IN;
    private boolean lastCellUp = true;
    private double reacquireOffsetDeg = 0;
    private double reacquireDirection = 1;
    private boolean shootingNow = false;
    private boolean resyncPressed = false;
    private double headingSign = 1;
    private double anchorRaw = 0;
    private double anchorTrue = 0;
    private int headingVotes = 0;
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

    private static final int HISTORY = 256;
    private final double[][] history = new double[HISTORY][5];
    private int historyCount = 0;
    private int historyNext = 0;
    private final ElapsedTime clock = new ElapsedTime();
    private double turretLastMovingAt = 0;
    private double turretSlowEstimatePos = 0.5;

    public TurretAim(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        loadHeadingSign();
        loadTurretDirection();
        Pose kept = PoseKeeper.readPinpoint(hardwareMap);
        PoseKeeper.Saved saved = PoseKeeper.loadFile();
        boolean memoryFresh = PoseMemory.getPose() != null && PoseMemory.secondsSinceSave() < AUTO_POSE_MAX_AGE_S;
        if (memoryFresh) {
            alliance = PoseMemory.getAlliance();
        } else if (saved != null) {
            alliance = saved.alliance;
        }

        follower = Constants.createFollower(hardwareMap);
        turret = hardwareMap.get(Servo.class, "turret");
        if (USE_TAG_LOCATING) {
            localizer = new TagLocalizer(hardwareMap);
        }

        Pose lastOurs = PoseMemory.getPose() != null ? PoseMemory.getPose() : saved != null ? saved.pose : null;
        boolean keptIsOurs = kept != null && lastOurs != null
                && Math.hypot(kept.getX() - lastOurs.getX(), kept.getY() - lastOurs.getY()) <= KEPT_MATCH_IN
                && (headingSign < 0 || Math.abs(wrapDegrees(Math.toDegrees(kept.getHeading() - lastOurs.getHeading()))) <= KEPT_MATCH_DEG);
        if (keptIsOurs) {
            startAt(headingSign < 0 ? new Pose(kept.getX(), kept.getY(), lastOurs.getHeading()) : kept);
            located = true;
            positionSource = "kept by the Pinpoint";
        } else if (kept != null) {
            startAt(new Pose(0, 0, 0));
            positionSource = "none";
        } else if (memoryFresh) {
            startAt(PoseMemory.getPose());
            located = true;
            positionSource = "carried over from the last OpMode";
        } else if (saved != null && saved.ageSeconds < SAVED_FILE_MAX_AGE_S) {
            startAt(saved.pose);
            located = true;
            positionSource = String.format("saved file (%.0f s old)", saved.ageSeconds);
        } else {
            startAt(new Pose(0, 0, 0));
        }

        if (USE_CAMERA_AIM) {
            vision = new Vision(hardwareMap, alliance);
        }
        turretLastMovingAt = clock.seconds() + TURRET_LIMIT_DEG / TURRET_SLOWEST_DEG_PER_S;
        turretSentDeg = TURRET_ZERO_POS * TURRET_DEGREES_PER_UNIT;
    }

    public void initLoop(boolean pickBlue, boolean pickRed, boolean resync) {
        Alliance before = alliance;
        if (pickBlue) {
            alliance = Alliance.BLUE;
        }
        if (pickRed) {
            alliance = Alliance.RED;
        }
        if (alliance != before) {
            historyCount = 0;
            lastBluePose = null;
            if (vision != null) {
                vision.setAlliance(alliance);
            }
        }
        follower.updatePose();
        updateFixedPose();
        resyncPressed = resync;
        resyncButton();
        Pose initPose = blueFramePose();
        remember(clock.seconds(), initPose.getX(), initPose.getY(), Math.toDegrees(initPose.getHeading()), turretActualDeg());
        tagRelocate(true);
        commandTurret(TURRET_ZERO_POS, 0);
        turretEstimatePos = TURRET_ZERO_POS;
        turretSlowEstimatePos = TURRET_ZERO_POS;
    }

    public void start() {
        loopTimer.reset();
        lastCameraSeenAt = clock.seconds();
    }

    public TurretAim(HardwareMap hardwareMap, Alliance startAlliance, Pose startPose) {
        this.hardwareMap = hardwareMap;
        loadHeadingSign();
        loadTurretDirection();
        alliance = startAlliance;
        follower = Constants.createFollower(hardwareMap);
        turret = hardwareMap.get(Servo.class, "turret");
        if (USE_TAG_LOCATING) {
            localizer = new TagLocalizer(hardwareMap);
        }
        startAt(startPose);
        located = true;
        positionSource = "auto start spot";
        if (USE_CAMERA_AIM) {
            vision = new Vision(hardwareMap, alliance);
        }
        turretLastMovingAt = clock.seconds() + TURRET_LIMIT_DEG / TURRET_SLOWEST_DEG_PER_S;
        turretSentDeg = TURRET_ZERO_POS * TURRET_DEGREES_PER_UNIT;
    }

    public void updateWhileFollowing(boolean shooting) {
        runUpdate(false, 0, shooting, false);
    }

    public void update(boolean resync, double turnStickX, boolean shooting, boolean searchButton) {
        follower.updatePose();
        runUpdate(resync, turnStickX, shooting, searchButton);
    }

    private void runUpdate(boolean resync, double turnStickX, boolean shooting, boolean searchButton) {
        updateFixedPose();
        resyncPressed = resync;
        resyncButton();
        double dt = updateVelocity();
        checkHeadingDirection(dt, turnStickX);
        tagRelocate(false);
        if (located) {
            PoseKeeper.save(currentPose(), alliance);
        }

        shootingNow = shooting;
        runSearchButton(searchButton);
        aimTurret(dt);
    }

    public boolean isAimKnown() {
        return aimKnown;
    }

    public boolean isAimed() {
        return aimKnown && turretCanReach && turretReady && cameraAgrees;
    }

    public boolean canReach() {
        return turretCanReach;
    }

    public boolean isTurretReady() {
        return turretReady;
    }

    public boolean cameraAgrees() {
        return cameraAgrees;
    }

    public double getDistanceToCell() {
        return distanceToCell;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public Follower getFollower() {
        return follower;
    }

    public Vision getVision() {
        return vision;
    }

    public boolean isLocated() {
        return located;
    }

    public String getAimSource() {
        return aimSource;
    }

    public void addInitTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.addData("alliance", alliance + "   (BLUE = X on Xbox / Square on PlayStation, RED = B on Xbox / Circle on PlayStation)");
        showInitCamera();
        showPosition();
    }

    public void addTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.addData("alliance", alliance);
        if (headingSign < 0) {
            telemetry.addLine("WARNING: Pinpoint heading is reversed (fixed in software for the turret only). Mount the Pinpoint sticker-side up so Pedro and the autos work.");
        }
        telemetry.addData("AIMING WITH", aimSource);
        telemetry.addData("camera", cameraStatus);
        if (!cameraFixMessage.isEmpty()) {
            telemetry.addLine(cameraFixMessage);
        }
        if (!Vision.getCameraFixMessage().isEmpty()) {
            telemetry.addLine(Vision.getCameraFixMessage());
        }
        showPosition();
        telemetry.addData("aiming at", (blueFrameCellIsRear() ? "REAR" : "AUDIENCE") + " cell");
        telemetry.addData("distance", "%.1f in", distanceToCell);
        telemetry.addData("robot speed", "%.0f in/s", Math.hypot(vx, vy));
        telemetry.addData("turret angle", "%.1f° %s", turretAngle,
                !aimKnown ? "(searching for the tags)"
                        : !turretCanReach ? "(AT LIMIT: can't aim yet - turn the robot a bit, or it will swing around)"
                        : !turretReady ? "moving" : !cameraAgrees ? "camera says still turning" : "ready");
        telemetry.addData("gamepad 2 Y", manualSearch ? "SEARCHING for tags (press Y again to stop)" : "press to sweep the turret once each way to find the tags");
    }

    public void stop() {
        if (localizer != null) {
            localizer.stop();
        }
        if (vision != null) {
            vision.stop();
        }
    }

    private void runSearchButton(boolean pressed) {
        if (pressed && !searchButtonWas) {
            manualSearch = !manualSearch;
            manualSearchEnds = 0;
            searchDirection = turretCommandDeg >= 0 ? 1 : -1;
            holdUntilCamera = false;
        }
        searchButtonWas = pressed;
    }

    private void resyncButton() {
        if (resyncPressed) {
            Pose spot = startSpot();
            Pose before = currentPose();
            setPoseEverywhere(spot);
            poseJumped(before, spot);
            vx = 0;
            vy = 0;
            omegaDeg = 0;
            located = true;
            positionSource = "re-synced on the start spot";
        }
    }

    private void tagRelocate(boolean always) {
        if (localizer == null) {
            return;
        }
        boolean still = always || (Math.hypot(vx, vy) < RELOCATE_MAX_SPEED_IN_S && Math.abs(omegaDeg) < RELOCATE_MAX_TURN_DEG_S);
        if (!still || !turretSteadyFor(RELOCATE_FRAME_MAX_AGE_S + RELOCATE_SETTLE_S)) {
            return;
        }
        double cameraYawDeg = TURRET_ZERO_FROM_FRONT_DEG + turretActualDeg();
        double yaw = Math.toRadians(cameraYawDeg);
        double cameraForward = TURRET_FORWARD_OF_CENTER_IN
                + CAMERA_ON_TURRET_FORWARD_IN * Math.cos(yaw) - CAMERA_ON_TURRET_LEFT_IN * Math.sin(yaw);
        double cameraLeft = TURRET_LEFT_OF_CENTER_IN
                + CAMERA_ON_TURRET_FORWARD_IN * Math.sin(yaw) + CAMERA_ON_TURRET_LEFT_IN * Math.cos(yaw);
        Pose found = localizer.locate(cameraYawDeg, cameraForward, cameraLeft);
        if (found != null) {
            Pose before = currentPose();
            setPoseEverywhere(found);
            poseJumped(before, found);
            located = true;
            positionSource = "AprilTags";
        }
    }

    private double updateVelocity() {
        Pose now = blueFramePose();
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

    private void poseJumped(Pose oldReal, Pose newReal) {
        Pose a = toBlueFrame(oldReal);
        Pose b = toBlueFrame(newReal);
        lastBluePose = b;
        signRefTime = -1e9;
        double turn = b.getHeading() - a.getHeading();
        double cos = Math.cos(turn);
        double sin = Math.sin(turn);
        for (int cell = 0; cell < 2; cell++) {
            double dx = cameraSpot[cell][0] - a.getX();
            double dy = cameraSpot[cell][1] - a.getY();
            cameraSpot[cell][0] = b.getX() + dx * cos - dy * sin;
            cameraSpot[cell][1] = b.getY() + dx * sin + dy * cos;
            double ddx = cameraSpotDistX[cell] - a.getX();
            double ddy = cameraSpotDistY[cell] - a.getY();
            cameraSpotDistX[cell] = b.getX() + ddx * cos - ddy * sin;
            cameraSpotDistY[cell] = b.getY() + ddx * sin + ddy * cos;
        }
        for (int i = 0; i < historyCount; i++) {
            double[] row = history[(historyNext - 1 - i + HISTORY) % HISTORY];
            double dx = row[1] - a.getX();
            double dy = row[2] - a.getY();
            row[1] = b.getX() + dx * cos - dy * sin;
            row[2] = b.getY() + dx * sin + dy * cos;
            row[3] += Math.toDegrees(turn);
        }
    }

    private boolean turretSteadyFor(double window) {
        double now = clock.seconds();
        if (turretLastMovingAt > now - window || historyCount == 0) {
            return false;
        }
        double current = turretActualDeg();
        double min = current;
        double max = current;
        boolean covered = false;
        for (int i = 0; i < historyCount; i++) {
            double[] row = history[(historyNext - 1 - i + HISTORY) % HISTORY];
            min = Math.min(min, row[4]);
            max = Math.max(max, row[4]);
            if (row[0] <= now - window) {
                covered = true;
                break;
            }
        }
        return covered && max - min <= RELOCATE_TURRET_STEADY_DEG;
    }

    private boolean turretSettledSince(double t) {
        return turretLastMovingAt <= t - TURRET_SETTLE_S && commandChangedAt <= t - TURRET_SETTLE_S;
    }

    private void aimTurret(double dt) {
        Pose p = blueFramePose();
        double headingDeg = Math.toDegrees(p.getHeading());
        double now = clock.seconds();
        remember(now, p.getX(), p.getY(), headingDeg, turretActualDeg());
        robotStill = Math.hypot(vx, vy) < TURRET_HOLD_MAX_SPEED_IN_PER_S && Math.abs(omegaDeg) < TURRET_HOLD_MAX_TURN_DEG_PER_S;
        if (located) {
            if (p.getY() > PIVOT_Y + CELL_SWITCH_HYSTERESIS_IN) {
                targetCell = 1;
            } else if (p.getY() < PIVOT_Y - CELL_SWITCH_HYSTERESIS_IN) {
                targetCell = 0;
            }
        }

        boolean cameraFresh = false;
        if (vision != null) {
            int wantedCell = located ? targetCell : -1;
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
                    + " hive tags (wrong alliance? in INIT press X/Square = blue, B/Circle = red)";
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

        if (located && cameraTrusted && got3d && seenCell >= 0) {
            correctPoseFromCamera(captureTime, seenCell, now);
            p = blueFramePose();
        }
        boolean validCell = seenCell == 0 || seenCell == 1;
        if (manualSearch) {
            if (cameraFresh && validCell) {
                manualSearch = false;
                holdUntilCamera = true;
                holdUntilCameraSince = now;
            } else {
                aimKnown = false;
                cameraAgrees = true;
                aimSource = "SEARCHING (gamepad 2 Y): one sweep each way, then back to odometry";
                if (sweepTurret(dt)) {
                    manualSearchEnds++;
                    if (manualSearchEnds >= 2) {
                        manualSearch = false;
                    }
                }
                return;
            }
        }
        if (holdUntilCamera && (now - holdUntilCameraSince > HOLD_FOR_CAMERA_S || lastSpotUpdateAt > holdUntilCameraSince)) {
            holdUntilCamera = false;
        }
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
                seenRise = haveCameraSpot[seenCell] ? cameraSpot[seenCell][2] : lastRise;
                aimSource = "LIMELIGHT (no 3D: turn on Full 3D)";
            }
            if (usable) {
                double seenDistX = thenTurretX + flat * Math.cos(bearing);
                double seenDistY = thenTurretY + flat * Math.sin(bearing);
                double seenAimX = seenDistX;
                double seenAimY = seenDistY;
                if (located) {
                    double deeper = FieldMap.CELL_AIM_FROM_PIVOT_IN
                            - (lastCellUp ? FieldMap.UP_TAGS_FROM_PIVOT_IN : FieldMap.DOWN_TAGS_FROM_PIVOT_IN);
                    seenAimY += seenCell == 1 ? deeper : -deeper;
                }
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
        } else if (cameraFresh && validCell && (!located || holdUntilCamera)) {
            aimKnown = false;
            cameraAgrees = true;
            turretCanReach = true;
            targetCell = seenCell;
            aimSource = "LIMELIGHT (found the tags: holding still so the turret settles)";
            smoothCommandDeg = turretCommandDeg;
            heldCommandDeg = turretCommandDeg;
            commandTurret(angleToPosition(turretCommandDeg), dt);
            return;
        } else if (located && haveCameraSpot[targetCell] && targetCell == lastCameraCell && secondsSinceSeen < CAMERA_MEMORY_S) {
            double[] spot = cameraSpot[targetCell];
            aimX = spot[0];
            aimY = spot[1];
            distX = cameraSpotDistX[targetCell];
            distY = cameraSpotDistY[targetCell];
            rise = spot[2];
            aimSource = "ODOMETRY (holding where the Limelight last saw the cell)";
        } else if (located) {
            aimX = PIVOT_X;
            aimY = targetCell == 1 ? PIVOT_Y + FieldMap.CELL_AIM_FROM_PIVOT_IN : PIVOT_Y - FieldMap.CELL_AIM_FROM_PIVOT_IN;
            distX = PIVOT_X;
            distY = rowY(targetCell, lastCellUp);
            rise = lastRise;
            addTrim = true;
            aimSource = cameraFresh ? "ODOMETRY (camera sees tags, waiting for a steady view)" : "ODOMETRY (camera can't see the tags)";
        } else if (lastCameraCell >= 0 && secondsSinceSeen < CAMERA_MEMORY_S) {
            double[] spot = cameraSpot[lastCameraCell];
            targetCell = lastCameraCell;
            aimX = spot[0];
            aimY = spot[1];
            distX = cameraSpotDistX[lastCameraCell];
            distY = cameraSpotDistY[lastCameraCell];
            rise = spot[2];
            aimSource = "LAST CAMERA VIEW (position unknown)";
        } else {
            aimKnown = false;
            cameraAgrees = true;
            aimSource = "SEARCHING: sweeping the turret to find the hive tags";
            sweepTurret(dt);
            return;
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
            aimSource = (located ? "ODOMETRY" : "LAST CAMERA VIEW") + " (camera lost the tags: looking around to find them)";
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


    private boolean sweepTurret(double dt) {
        double limit = Math.min(SEARCH_LIMIT_DEG, TURRET_LIMIT_DEG);
        double next = turretCommandDeg + searchDirection * SEARCH_SPEED_DEG_PER_S * dt;
        boolean turnedAround = false;
        if (next >= limit) {
            next = limit;
            turnedAround = searchDirection > 0;
            searchDirection = -1;
        } else if (next <= -limit) {
            next = -limit;
            turnedAround = searchDirection < 0;
            searchDirection = 1;
        }
        turretAngle = next;
        turretCommandDeg = next;
        heldCommandDeg = next;
        smoothCommandDeg = next;
        turretCanReach = false;
        distanceToCell = 0;
        commandTurret(angleToPosition(next), dt);
        return turnedAround;
    }



    private double rowY(int cell, boolean up) {
        double fromPivot = up ? FieldMap.UP_TAGS_FROM_PIVOT_IN : FieldMap.DOWN_TAGS_FROM_PIVOT_IN;
        return cell == 1 ? PIVOT_Y + fromPivot : PIVOT_Y - fromPivot;
    }

    private void correctPoseFromCamera(double captureTime, int cell, double now) {
        boolean full = vision.hasMeasuredScale();
        double angle = full ? vision.getScaledAngle() : vision.getAngle();
        double flat = full ? vision.getScaledFlatDistance() : vision.getFlatDistance();
        double[] then = poseAt(captureTime);
        double thenH = Math.toRadians(then[2]);
        double bearing = Math.toRadians(then[2] + TURRET_ZERO_FROM_FRONT_DEG + then[3] - angle);
        double thenTurretX = then[0] + TURRET_FORWARD_OF_CENTER_IN * Math.cos(thenH) - TURRET_LEFT_OF_CENTER_IN * Math.sin(thenH);
        double thenTurretY = then[1] + TURRET_FORWARD_OF_CENTER_IN * Math.sin(thenH) + TURRET_LEFT_OF_CENTER_IN * Math.cos(thenH);
        double seenX = thenTurretX + flat * Math.cos(bearing);
        double seenY = thenTurretY + flat * Math.sin(bearing);

        if (full) {
            lastRise = vision.getScaledRise();
            double upDownSplit = (FieldMap.UP_TAGS_HEIGHT_IN + FieldMap.DOWN_TAGS_HEIGHT_IN) / 2 - Vision.CAMERA_HEIGHT_IN;
            lastCellUp = lastRise > upDownSplit;
        }

        double errorX = PIVOT_X - seenX;
        double errorY = rowY(cell, lastCellUp) - seenY;
        double errorSize = Math.hypot(errorX, errorY);
        if (errorSize > MAX_CAMERA_CORRECTION_IN) {
            lastCameraErrorIn = errorSize;
            if (!full || errorSize > BIG_ERROR_MAX_IN) {
                bigErrorCount = 0;
                return;
            }
            if (bigErrorCount > 0 && Math.hypot(errorX - bigErrorX, errorY - bigErrorY) <= BIG_ERROR_AGREE_IN) {
                bigErrorCount++;
            } else {
                bigErrorCount = 1;
            }
            bigErrorX = errorX;
            bigErrorY = errorY;
            if (bigErrorCount >= BIG_ERROR_FRAMES) {
                Pose before = currentPose();
                Pose blue = toBlueFrame(before);
                Pose corrected = fromBlueFrame(new Pose(blue.getX() + errorX, blue.getY() + errorY, blue.getHeading()));
                setPoseEverywhere(corrected);
                poseJumped(before, corrected);
                lastCorrectionAt = now;
                bigErrorCount = 0;
                positionSource = "fixed by the camera (it was " + Math.round(errorSize) + " in off)";
            }
            return;
        }
        bigErrorCount = 0;
        if (!full) {
            double along = errorX * Math.cos(bearing) + errorY * Math.sin(bearing);
            errorX -= along * Math.cos(bearing);
            errorY -= along * Math.sin(bearing);
        }
        lastCameraErrorIn = Math.hypot(errorX, errorY);
        if (now - lastCorrectionAt < POSE_CORRECTION_PERIOD_S) {
            return;
        }
        double stepX = errorX * POSE_CORRECTION_GAIN;
        double stepY = errorY * POSE_CORRECTION_GAIN;
        double step = Math.hypot(stepX, stepY);
        if (step < POSE_CORRECTION_MIN_STEP_IN) {
            return;
        }
        if (step > POSE_CORRECTION_MAX_STEP_IN) {
            stepX *= POSE_CORRECTION_MAX_STEP_IN / step;
            stepY *= POSE_CORRECTION_MAX_STEP_IN / step;
        }
        Pose before = currentPose();
        Pose blue = toBlueFrame(before);
        Pose corrected = fromBlueFrame(new Pose(blue.getX() + stepX, blue.getY() + stepY, blue.getHeading()));
        setPoseEverywhere(corrected);
        poseJumped(before, corrected);
        lastCorrectionAt = now;
    }

    private double guessFlatDistance(double fromX, double fromY, int cell) {
        if (cell >= 0 && haveCameraSpot[cell]) {
            return Math.hypot(cameraSpot[cell][0] - fromX, cameraSpot[cell][1] - fromY);
        }
        if (located) {
            double mapY = targetCell == 1 ? PIVOT_Y + FieldMap.CELL_AIM_FROM_PIVOT_IN : PIVOT_Y - FieldMap.CELL_AIM_FROM_PIVOT_IN;
            return Math.hypot(PIVOT_X - fromX, mapY - fromY);
        }
        return 60;
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
        int tags = vision.getTagsUsed();
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
            checkRefTags = tags;
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
            if (headingCameraVotes >= HEADING_CAMERA_VOTES) {
                headingFromCamera = true;
            }
        }
        if (headingCameraVotes <= -HEADING_CAMERA_VOTES) {
            headingFromCamera = true;
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

    private void showInitCamera() {
        if (vision == null) {
            telemetry.addData("camera", "off (USE_CAMERA_AIM is false)");
            return;
        }
        vision.update(Math.toDegrees(blueFramePose().getHeading()) + TURRET_ZERO_FROM_FRONT_DEG + turretActualDeg(), -1);
        if (vision.getRawTagIds().isEmpty()) {
            telemetry.addData("camera", "no tags in view (Limelight pipeline %d)", Vision.PIPELINE);
        } else if (!vision.sawOurAllianceTag()) {
            telemetry.addData("camera", "sees tags %s: NOT %s tags (%s hive is %s) - wrong alliance selected?",
                    vision.getRawTagIds(), alliance, alliance, alliance == Alliance.BLUE ? "38-45" : "30-37");
        } else {
            telemetry.addData("camera", "sees tags %s: OK, %s %s",
                    vision.getRawTagIds(), alliance,
                    !vision.hasFreshTag() ? "(no usable cell yet)"
                            : (alliance == Alliance.RED ? vision.getTargetCluster() == 0 : vision.getTargetCluster() == 1) ? "REAR cell" : "AUDIENCE cell");
        }
    }

    private boolean blueFrameCellIsRear() {
        return alliance == Alliance.RED ? targetCell == 0 : targetCell == 1;
    }

    private Pose startSpot() {
        double r = ROBOT_HALF_IN;
        double x = FieldMap.CENTER + r + 1;
        double y = FIELD - r;
        if (alliance == Alliance.RED) {
            return new Pose(FIELD - x, FIELD - y, Math.toRadians(90));
        }
        return new Pose(x, y, Math.toRadians(-90));
    }

    private Pose blueFramePose() {
        return toBlueFrame(currentPose());
    }

    private void startAt(Pose pose) {
        follower.setStartingPose(pose);
        resetFixedPose(pose);
    }

    private void setPoseEverywhere(Pose pose) {
        if (!isGood(pose)) {
            return;
        }
        follower.setPose(pose);
        resetFixedPose(pose);
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

    private void updateFixedPose() {
        Pose raw = follower.getPose();
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

    private static double wrapDegreesRad(double radians) {
        return Math.toRadians(wrapDegrees(Math.toDegrees(radians)));
    }

    private Pose currentPose() {
        Pose raw = follower.getPose();
        double rawHeading = isGood(raw) ? raw.getHeading() : lastRawH;
        return new Pose(fixedX, fixedY, anchorTrue + headingSign * (rawHeading - anchorRaw));
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

    private void checkHeadingDirection(double dt, double turnStick) {
        Pose rawPose = follower.getPose();
        if (!isGood(rawPose)) {
            return;
        }
        double raw = rawPose.getHeading();
        if (poseWasSet || Double.isNaN(lastRawHeading)) {
            poseWasSet = false;
            lastRawHeading = raw;
            return;
        }
        gyroTurnDeg += wrapDegrees(Math.toDegrees(raw - lastRawHeading));
        double rawRate = wrapDegrees(Math.toDegrees(raw - lastRawHeading)) / Math.max(dt, 1e-3);
        lastRawHeading = raw;
        if (HEADING_SIGN_SETTING != 0 || headingFromCamera || Math.abs(turnStick) < HEADING_CHECK_STICK || Math.abs(rawRate) < HEADING_CHECK_MIN_DEG_PER_S) {
            return;
        }
        headingVotes += turnStick * rawRate < 0 ? 1 : -1;
        headingVotes = Math.max(-2 * HEADING_CHECK_VOTES, Math.min(2 * HEADING_CHECK_VOTES, headingVotes));
        if (headingVotes >= HEADING_CHECK_VOTES) {
            setHeadingSign(1);
        } else if (headingVotes <= -HEADING_CHECK_VOTES) {
            setHeadingSign(-1);
        }
    }

    private Pose fromBlueFrame(Pose blue) {
        if (alliance == Alliance.RED) {
            return new Pose(FIELD - blue.getX(), FIELD - blue.getY(), blue.getHeading() + Math.PI);
        }
        return blue;
    }

    private Pose toBlueFrame(Pose real) {
        if (alliance == Alliance.RED) {
            return new Pose(FIELD - real.getX(), FIELD - real.getY(), real.getHeading() - Math.PI);
        }
        return real;
    }

    private void showPosition() {
        Pose p = currentPose();
        telemetry.addData("position", located ? "known (" + positionSource + ")"
                : "UNKNOWN: back the robot up to the hive so the camera sees 2 tags, or put it on the auto start spot and press BACK/SHARE");
        telemetry.addData("robot", "x %.1f  y %.1f  h %.1f°", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }


    public static TurretAim createAuto(HardwareMap hardwareMap, Alliance alliance, Pose startPose) {
        return new TurretAim(hardwareMap, startPose, alliance);
    }

    private TurretAim(HardwareMap hardwareMap, Pose startPose, Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.alliance = alliance;
        loadHeadingSign();
        loadTurretDirection();
        turret = hardwareMap.get(Servo.class, "turret");
        resetFixedPose(startPose);
        located = true;
        positionSource = "OpMode Pinpoint pose";
        if (USE_CAMERA_AIM) {
            vision = new Vision(hardwareMap, alliance);
        }
        turretLastMovingAt = clock.seconds() + TURRET_LIMIT_DEG / TURRET_SLOWEST_DEG_PER_S;
        turretSentDeg = TURRET_ZERO_POS * TURRET_DEGREES_PER_UNIT;
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
            int wantedCell = located ? targetCell : -1;
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
                    + " hive tags (wrong alliance? in INIT press X/Square = blue, B/Circle = red)";
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
                seenRise = haveCameraSpot[seenCell] ? cameraSpot[seenCell][2] : lastRise;
                aimSource = "LIMELIGHT (no 3D: turn on Full 3D)";
            }
            if (usable) {
                double seenDistX = thenTurretX + flat * Math.cos(bearing);
                double seenDistY = thenTurretY + flat * Math.sin(bearing);
                double seenAimX = seenDistX;
                double seenAimY = seenDistY;
                if (located) {
                    double deeper = FieldMap.CELL_AIM_FROM_PIVOT_IN
                            - (lastCellUp ? FieldMap.UP_TAGS_FROM_PIVOT_IN : FieldMap.DOWN_TAGS_FROM_PIVOT_IN);
                    seenAimY += seenCell == 1 ? deeper : -deeper;
                }
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
        } else if (located && haveCameraSpot[targetCell] && targetCell == lastCameraCell && secondsSinceSeen < CAMERA_MEMORY_S) {
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
            distY = rowY(targetCell, lastCellUp);
            rise = lastRise;
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
            aimSource = (located ? "ODOMETRY" : "LAST CAMERA VIEW") + " (camera lost the tags: looking around to find them)";
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

    public void addTelemetryAuto(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.addData("alliance", alliance);
        if (headingSign < 0) {
            telemetry.addLine("WARNING: Pinpoint heading is reversed (fixed in software for the turret only). Mount the Pinpoint sticker-side up so Pedro and the autos work.");
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
        telemetry.addData("position", positionSource);
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
}
