package org.firstinspires.ftc.teamcode.Neel;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AutoBlue", group = "Glitch")
public class Autoblue extends LinearOpMode {

    public static double FIELD = 141.5;
    public static double CENTER = FIELD / 2;
    public static double PIVOT_X = CENTER + 12.75;
    public static double PIVOT_Y = CENTER;
    public static double CELL_AIM_FROM_PIVOT_IN = 16.1;
    public static double UP_TAGS_FROM_PIVOT_IN = 13.85;

    public static double ROBOT_HALF_IN = 9;

    public static double START_X, START_Y, START_H;
    public static double TURN_X, TURN_Y, TURN_H;
    public static double BALLS_START_X, BALLS_START_Y, BALLS_START_H;
    public static double BALLS_END_X, BALLS_END_Y, BALLS_END_H;
    public static double BACK_OFF_X, BACK_OFF_Y, BACK_OFF_H;
    public static double CHECK_X, CHECK_Y, CHECK_H;
    public static double VIA_SIDE_X;
    public static double SHOOT_AUDIENCE_X, SHOOT_AUDIENCE_Y, SHOOT_AUDIENCE_H;
    public static double SHOOT_REAR_X, SHOOT_REAR_Y, SHOOT_REAR_H;
    public static double SHOOT_TAG_DISTANCE_IN = 36;
    public static double CHECK_TAG_DISTANCE_IN = 16;
    public static double CHECK_SIDE_SHIFT_IN = 2;

    public static double DRIVE_MAX_POWER = 0.7;
    public static double INTAKE_MAX_POWER = 0.3;
    public static double PATH_TIMEOUT_S = 4;
    public static double INTAKE_EXTRA_S = 0.5;
    public static double INTAKE_POWER = 0.5;
    public static double INTAKE_POWER_WHILE_SHOOTING = 0;

    public static double FIRST_SHOOT_TIME_S = 4;
    public static double SECOND_SHOOT_TIME_S = 4;
    public static double AIM_TIMEOUT_S = 1.5;
    public static double TILT_LOOK_TIME_S = 1.0;
    public static double TILT_MIN_DIFF_IN = 4.0;
    public static double CAMERA_HEIGHT_IN = 13;
    public static double UP_TAGS_HEIGHT_IN = 48.48;
    public static double DOWN_TAGS_HEIGHT_IN = 34.22;
    public static int DEFAULT_CELL = 1;

    public static double SHOOTER_KF = 0.00042;
    public static int DEFAULT_VELOCITY = 900;
    public static int MAX_VELOCITY = 2200;
    public static double READY_AT = 0.97;
    public static double DROP_FROM_PEAK = 0.03;
    public static double MIN_PAUSE_MS = 150;
    public static double MAX_PAUSE_MS = 500;
    public static double SPIN_UP_MS = 1200;
    public static double TRANSFER_POWER = 1.0;

    public static double TAG_HOLD_MS = 300;
    public static double REAL_TAG_MM = 82.55;
    public static double LIMELIGHT_MARKER_MM = 101.6;
    public static double CAMERA_PITCH_DEG = 60;
    public static double CAMERA_RIGHT_OF_CENTER_IN = 5.0 / 2.54;
    public static double CAMERA_FORWARD_OF_CENTER_IN = 0;
    public static double AIM_TRIM_DEG = 0;

    public static double AIM_KP = 0.015;
    public static double AIM_KS = 0.06;
    public static double AIM_MAX_TURN = 0.5;
    public static double AIM_TOLERANCE = 0.75;
    public static double AIM_UPDATE_MAX_RATE = 5;
    public static double AIM_STILL_MS = 120;
    private final ElapsedTime stillTimer = new ElapsedTime();
    public static double AIM_SETTLE_MS = 250;

    private Follower follower;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx outtake;
    private DcMotorEx intake;
    private CRServo transfer;
    private Limelight3A limelight;

    private final ElapsedTime lastSeenTimer = new ElapsedTime();
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private final ElapsedTime headingTimer = new ElapsedTime();

    int targetCluster = -1;
    boolean tagVisible = false;
    boolean freshFrame = false;
    double lastFrameSignature = Double.NaN;
    int tagId = -1;
    int tagsUsed = 0;
    double distanceToTarget = -1;
    double angleToTarget = 0;

    boolean hasAimHeading = false;
    double aimHeading = 0;
    double lastHeading = 0;
    double turnRate = 0;

    int goalVelocity = 0;
    double currentVelocity = 0;
    double peakVelocity = 0;
    boolean isFeeding = false;

    String step = "init";
    int higherCell = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        setUpPoses();
        initHardware();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pose(START_X, START_Y, START_H));
        lastHeading = currentHeading();

        while (opModeInInit()) {
            targetCluster = -1;
            updateHeading();
            updateVision();
            telemetry.addData("alliance", "BLUE");
            telemetry.addData("higher cell now", cellName(readHigherCell()));
            showCameraCheck();
            telemetry.addData("start", "x %.1f  y %.1f  h %.0f°", START_X, START_Y, START_H);
            telemetry.addLine("Ready. Press START.");
            telemetry.update();
        }

        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        step = "to first shot";
        driveTo(pose(SHOOT_REAR_X, SHOOT_REAR_Y, SHOOT_REAR_H), DRIVE_MAX_POWER, false, true);

        step = "first shots";
        aimAndShoot(1, FIRST_SHOOT_TIME_S);

        step = "to balls";
        driveTo(pose(TURN_X, TURN_Y, TURN_H), DRIVE_MAX_POWER, false, false);
        driveTo(pose(BALLS_START_X, BALLS_START_Y, BALLS_START_H), DRIVE_MAX_POWER, false, false);

        step = "intaking";
        intake.setPower(INTAKE_POWER);
        driveTo(pose(BALLS_END_X, BALLS_END_Y, BALLS_END_H), INTAKE_MAX_POWER, true, false);
        ElapsedTime extra = new ElapsedTime();
        while (opModeIsActive() && extra.seconds() < INTAKE_EXTRA_S) {
            telemetry.addData("step", step);
            telemetry.update();
        }

        step = "backing off";
        driveTo(pose(BACK_OFF_X, BACK_OFF_Y, BACK_OFF_H), DRIVE_MAX_POWER, true, false);
        intake.setPower(0);

        step = "checking tilt";
        driveTo(pose(CHECK_X, CHECK_Y + 10, CHECK_H), DRIVE_MAX_POWER, false, true);
        driveTo(pose(CHECK_X, CHECK_Y, CHECK_H), DRIVE_MAX_POWER, false, true);
        higherCell = lookForHigherCell(TILT_LOOK_TIME_S);
        int cell = higherCell >= 0 ? higherCell : DEFAULT_CELL;

        step = "to " + cellName(cell) + " cell";
        if (cell == 0) {
            driveTo(pose(CHECK_X, CHECK_Y + 8, CHECK_H), DRIVE_MAX_POWER, false, true);
            driveTo(pose(VIA_SIDE_X, CHECK_Y + 8, CHECK_H), DRIVE_MAX_POWER, false, true);
            driveTo(pose(VIA_SIDE_X, SHOOT_AUDIENCE_Y, SHOOT_AUDIENCE_H), DRIVE_MAX_POWER, false, true);
            driveTo(pose(SHOOT_AUDIENCE_X, SHOOT_AUDIENCE_Y, SHOOT_AUDIENCE_H), DRIVE_MAX_POWER, false, true);
        } else {
            driveTo(pose(SHOOT_REAR_X, SHOOT_REAR_Y, SHOOT_REAR_H), DRIVE_MAX_POWER, false, true);
        }

        step = "shooting " + cellName(cell) + " cell";
        aimAndShoot(cell, SECOND_SHOOT_TIME_S);

        finish("done");
    }

    private void setUpPoses() {
        double r = ROBOT_HALF_IN;
        START_X = CENTER + r + 1;
        START_Y = FIELD - r;
        START_H = -90;

        SHOOT_REAR_X = PIVOT_X;
        SHOOT_REAR_Y = PIVOT_Y + UP_TAGS_FROM_PIVOT_IN + SHOOT_TAG_DISTANCE_IN + CAMERA_FORWARD_OF_CENTER_IN;
        SHOOT_REAR_H = -90;
        SHOOT_AUDIENCE_X = PIVOT_X;
        SHOOT_AUDIENCE_Y = PIVOT_Y - UP_TAGS_FROM_PIVOT_IN - SHOOT_TAG_DISTANCE_IN - CAMERA_FORWARD_OF_CENTER_IN;
        SHOOT_AUDIENCE_H = 90;

        TURN_X = 100;
        TURN_Y = FIELD - r - 9;
        TURN_H = 90;

        BALLS_START_X = FIELD - r - 0.5;
        BALLS_START_Y = FIELD - r - 9;
        BALLS_START_H = 90;

        BALLS_END_X = FIELD - r - 0.5;
        BALLS_END_Y = FIELD - r - 0.5;
        BALLS_END_H = 90;

        BACK_OFF_X = FIELD - r - 0.5;
        BACK_OFF_Y = FIELD - r - 9;
        BACK_OFF_H = 90;

        CHECK_X = PIVOT_X - CHECK_SIDE_SHIFT_IN;
        CHECK_Y = PIVOT_Y + UP_TAGS_FROM_PIVOT_IN + CHECK_TAG_DISTANCE_IN + CAMERA_FORWARD_OF_CENTER_IN;
        CHECK_H = -90;

        VIA_SIDE_X = 113;
    }

    private double aimPointY(int cluster) {
        return cluster == 0 ? PIVOT_Y - CELL_AIM_FROM_PIVOT_IN : PIVOT_Y + CELL_AIM_FROM_PIVOT_IN;
    }

    private double headingToAimPoint(double x, double y, int cluster) {
        return Math.toDegrees(Math.atan2(aimPointY(cluster) - y, PIVOT_X - x));
    }

    private void finish(String message) {
        step = message;
        stopAll();
        limelight.stop();
        telemetry.addData("step", step);
        telemetry.addData("higher cell", cellName(higherCell));
        telemetry.update();
    }

    private Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    private void driveTo(Pose target, double maxPower, boolean intakeOn, boolean preSpin) {
        follower.update();
        Pose from = follower.getPose();
        double endHeading = from.getHeading()
                + Math.toRadians(wrapDegrees(Math.toDegrees(target.getHeading() - from.getHeading())));
        target = new Pose(target.getX(), target.getY(), endHeading);
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(from, target))
                .setLinearHeadingInterpolation(from.getHeading(), target.getHeading())
                .build();
        follower.followPath(path, maxPower, true);

        goalVelocity = preSpin ? Math.min(DEFAULT_VELOCITY, MAX_VELOCITY) : 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_S) {
            follower.update();
            runOuttakeMotor();
            if (intakeOn) {
                intake.setPower(INTAKE_POWER);
            }
            telemetry.addData("step", step);
            telemetry.addData("pose", "x %.1f  y %.1f  h %.1f°", follower.getPose().getX(),
                    follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
        follower.breakFollowing();
        setDrive(0, 0, 0);
        lastHeading = currentHeading();
        headingTimer.reset();
    }

    private int lookForHigherCell(double seconds) {
        int[] votes = new int[2];
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            int reading = readHigherCell();
            if (reading >= 0) {
                votes[reading]++;
            }
            telemetry.addData("step", step);
            telemetry.addData("tilt votes", "audience %d  rear %d", votes[0], votes[1]);
            telemetry.update();
        }
        if (votes[0] == 0 && votes[1] == 0) {
            return -1;
        }
        return votes[1] >= votes[0] ? 1 : 0;
    }

    private double[][] measureClusterHeights() {
        double[][] out = new double[2][2];
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return out;
        }
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (!isTargetTag(tag.getFiducialId())) {
                continue;
            }
            double[] p = tagPositionInches(tag);
            if (p[2] <= 1) {
                continue;
            }
            int c = clusterOf(tag.getFiducialId());
            out[c][0] += -p[1] * Math.cos(pitch) + p[2] * Math.sin(pitch);
            out[c][1]++;
        }
        for (int c = 0; c < 2; c++) {
            if (out[c][1] > 0) {
                out[c][0] /= out[c][1];
            }
        }
        return out;
    }

    private int readHigherCell() {
        double[][] h = measureClusterHeights();
        boolean seeAudience = h[0][1] > 0;
        boolean seeRear = h[1][1] > 0;

        if (seeAudience && seeRear) {
            double difference = h[1][0] - h[0][0];
            if (Math.abs(difference) < TILT_MIN_DIFF_IN) {
                return -1;
            }
            return difference > 0 ? 1 : 0;
        }

        if (CAMERA_HEIGHT_IN > 0 && (seeAudience || seeRear)) {
            int c = seeRear ? 1 : 0;
            double height = CAMERA_HEIGHT_IN + h[c][0];
            double split = (UP_TAGS_HEIGHT_IN + DOWN_TAGS_HEIGHT_IN) / 2;
            boolean thisCellUp = height > split;
            return thisCellUp ? c : 1 - c;
        }
        return -1;
    }

    private void showCameraCheck() {
        double[][] h = measureClusterHeights();
        if (CAMERA_HEIGHT_IN <= 0 || CAMERA_PITCH_DEG == 0) {
            telemetry.addLine("SET CAMERA_HEIGHT_IN and CAMERA_PITCH_DEG for a reliable tilt check");
        }
        if (h[1][1] > 0) {
            telemetry.addData("rear tags height", "%.1f in (should be %.1f at match start)",
                    CAMERA_HEIGHT_IN + h[1][0], UP_TAGS_HEIGHT_IN);
        }
        if (h[0][1] > 0) {
            telemetry.addData("audience tags height", "%.1f in (should be %.1f at match start)",
                    CAMERA_HEIGHT_IN + h[0][0], DOWN_TAGS_HEIGHT_IN);
        }
    }

    private String cellName(int cell) {
        if (cell == 0) return "AUDIENCE";
        if (cell == 1) return "REAR";
        return "unknown";
    }

    private void aimAndShoot(int cluster, double shootSeconds) {
        targetCluster = cluster;
        hasAimHeading = false;
        goalVelocity = 0;

        ElapsedTime aimTimer = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        while (opModeIsActive() && aimTimer.seconds() < AIM_TIMEOUT_S) {
            updateHeading();
            updateVision();
            setGoalVelocity();
            runOuttakeMotor();
            boolean aimed = turnTowardCell();
            if (!aimed) {
                settled.reset();
            } else if (settled.milliseconds() >= AIM_SETTLE_MS) {
                break;
            }
            telemetry.addData("step", step + " (aiming)");
            telemetry.update();
        }

        ElapsedTime shootTimer = new ElapsedTime();
        spinUpTimer.reset();
        pauseTimer.reset();
        isFeeding = false;
        intake.setPower(INTAKE_POWER_WHILE_SHOOTING);
        while (opModeIsActive() && shootTimer.seconds() < shootSeconds) {
            updateHeading();
            updateVision();
            setGoalVelocity();
            runOuttakeMotor();
            turnTowardCell();
            runTransfer();
            telemetry.addData("step", step);
            telemetry.addData("shooting", "%.1f / %.1f s", shootTimer.seconds(), shootSeconds);
            telemetry.update();
        }

        transfer.setPower(0);
        isFeeding = false;
        intake.setPower(0);
        setDrive(0, 0, 0);
        targetCluster = -1;
    }

    private void updateHeading() {
        double heading = currentHeading();
        double dt = Math.max(headingTimer.seconds(), 1e-3);
        turnRate = Math.abs(wrapDegrees(heading - lastHeading)) / dt;
        lastHeading = heading;
        headingTimer.reset();
    }

    private double currentHeading() {
        follower.update();
        return Math.toDegrees(follower.getPose().getHeading());
    }

    private boolean turnTowardCell() {
        double heading = lastHeading;

        if (turnRate >= AIM_UPDATE_MAX_RATE) {
            stillTimer.reset();
        }
        if (tagVisible && (!hasAimHeading || stillTimer.milliseconds() >= AIM_STILL_MS)) {
            aimHeading = wrapDegrees(heading - angleToTarget);
            hasAimHeading = true;
            telemetry.addData("aimingAt", "CELL CENTER (camera, " + tagsUsed + " tag" + (tagsUsed == 1 ? ")" : "s)"));
        } else if (!tagVisible && targetCluster >= 0) {
            Pose p = follower.getPose();
            aimHeading = headingToAimPoint(p.getX(), p.getY(), targetCluster);
            hasAimHeading = true;
            telemetry.addData("aimingAt", "CELL CENTER (odometry, tag not in view)");
        }

        if (!hasAimHeading) {
            setDrive(0, 0, 0);
            telemetry.addData("aim", "waiting for tag");
            return false;
        }

        double error = wrapDegrees(heading - aimHeading);
        double rx = 0;
        if (Math.abs(error) > AIM_TOLERANCE) {
            rx = AIM_KP * error + Math.signum(error) * AIM_KS;
            rx = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, rx));
        }
        setDrive(0, 0, rx);
        telemetry.addData("aimError", "%.2f°", error);
        return Math.abs(error) <= AIM_TOLERANCE;
    }

    private void setGoalVelocity() {
        if (tagVisible && distanceToTarget > 32 && distanceToTarget < 114) {
            int fromDistance = (int) Math.round(4.086 * distanceToTarget + 666);
            goalVelocity = Math.min(fromDistance, MAX_VELOCITY);
        } else if (!tagVisible && targetCluster >= 0) {
            Pose p = follower.getPose();
            double tagsY = targetCluster == 0 ? PIVOT_Y - UP_TAGS_FROM_PIVOT_IN : PIVOT_Y + UP_TAGS_FROM_PIVOT_IN;
            double flat = Math.hypot(PIVOT_X - p.getX(), tagsY - p.getY());
            double rise = UP_TAGS_HEIGHT_IN - CAMERA_HEIGHT_IN;
            double odometryDistance = Math.hypot(flat, rise);
            if (odometryDistance > 32 && odometryDistance < 114) {
                goalVelocity = Math.min((int) Math.round(4.086 * odometryDistance + 666), MAX_VELOCITY);
            } else if (goalVelocity == 0) {
                goalVelocity = Math.min(DEFAULT_VELOCITY, MAX_VELOCITY);
            }
        } else if (goalVelocity == 0) {
            goalVelocity = Math.min(DEFAULT_VELOCITY, MAX_VELOCITY);
        }
        telemetry.addData("goalVelocity", goalVelocity);
    }

    private void runOuttakeMotor() {
        currentVelocity = Math.abs(outtake.getVelocity());
        double power = 0;
        if (goalVelocity > 0) {
            power = currentVelocity < goalVelocity ? 1 : Math.min(1, SHOOTER_KF * goalVelocity);
        }
        outtake.setPower(power);
        telemetry.addData("curVelocity", "%.0f", currentVelocity);
    }

    private void runTransfer() {
        if (goalVelocity <= 0) {
            isFeeding = false;
        } else if (isFeeding) {
            peakVelocity = Math.max(peakVelocity, currentVelocity);
            if (currentVelocity < peakVelocity * (1 - DROP_FROM_PEAK)) {
                isFeeding = false;
                pauseTimer.reset();
            }
        } else {
            boolean atSpeed = currentVelocity >= goalVelocity * READY_AT;
            boolean spunUp = spinUpTimer.milliseconds() >= SPIN_UP_MS;
            boolean pausedLongEnough = pauseTimer.milliseconds() >= MAX_PAUSE_MS;
            if ((atSpeed || (spunUp && pausedLongEnough)) && pauseTimer.milliseconds() >= MIN_PAUSE_MS) {
                isFeeding = true;
                peakVelocity = currentVelocity;
            }
        }
        transfer.setPower(isFeeding ? TRANSFER_POWER : 0);
        telemetry.addData("transfer", isFeeding ? "FEEDING" : "waiting");
    }

    private void updateVision() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> seen = new ArrayList<>();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (isTargetTag(id) && (targetCluster < 0 || clusterOf(id) == targetCluster)) {
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

        freshFrame = false;

        if (best != null) {
            tagId = best.getFiducialId();
            int cluster = clusterOf(tagId);
            double[] p = tagPositionInches(best);
            double pitch = Math.toRadians(CAMERA_PITCH_DEG);

            if (p[2] > 1) {
                double[] middle = findClusterMiddle(best, p, seen);
                distanceToTarget = Math.sqrt(middle[0] * middle[0] + middle[1] * middle[1] + middle[2] * middle[2]);

                double outwardDeg = cluster == 0 ? -90 : 90;
                double[] outward = fieldDirectionInCamera(outwardDeg);
                double depth = CELL_AIM_FROM_PIVOT_IN - UP_TAGS_FROM_PIVOT_IN;
                double ax = middle[0] + depth * outward[0];
                double ay = middle[1] + depth * outward[1];
                double az = middle[2] + depth * outward[2];

                double xRobot = ax + CAMERA_RIGHT_OF_CENTER_IN;
                double forwardRobot = az * Math.cos(pitch) + ay * Math.sin(pitch) + CAMERA_FORWARD_OF_CENTER_IN;
                angleToTarget = Math.toDegrees(Math.atan2(xRobot, forwardRobot)) + AIM_TRIM_DEG;
            } else {
                tagsUsed = 0;
                angleToTarget = best.getTargetXDegrees() + AIM_TRIM_DEG;
            }

            double signature = best.getTargetXDegrees() * 1000 + best.getTargetArea();
            freshFrame = signature != lastFrameSignature;
            lastFrameSignature = signature;

            tagVisible = true;
            lastSeenTimer.reset();
        } else if (lastSeenTimer.milliseconds() > TAG_HOLD_MS) {
            tagVisible = false;
            tagId = -1;
            tagsUsed = 0;
            distanceToTarget = -1;
        }

        telemetry.addData("tag", tagVisible ? tagId + " (" + tagsUsed + " used)" : "not seen");
        telemetry.addData("distance", "%.1f in", distanceToTarget);
        telemetry.addData("angle", "%.2f°", angleToTarget);
    }

    private double[] fieldDirectionInCamera(double fieldAngleDeg) {
        double relative = Math.toRadians(fieldAngleDeg - lastHeading);
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double right = -Math.sin(relative);
        double forward = Math.cos(relative);
        return new double[]{right, forward * Math.sin(pitch), forward * Math.cos(pitch)};
    }

    private double[] findClusterMiddle(LLResultTypes.FiducialResult best, double[] p,
                                       List<LLResultTypes.FiducialResult> seen) {
        double bestX = getTagFieldOffsetInches(best.getFiducialId());

        LLResultTypes.FiducialResult partner = null;
        double[] partnerPos = null;
        double widestGap = 0;
        for (LLResultTypes.FiducialResult tag : seen) {
            if (tag == best || clusterOf(tag.getFiducialId()) != clusterOf(best.getFiducialId())) {
                continue;
            }
            double gap = Math.abs(getTagFieldOffsetInches(tag.getFiducialId()) - bestX);
            double[] q = tagPositionInches(tag);
            if (q[2] > 1 && gap > widestGap) {
                partner = tag;
                partnerPos = q;
                widestGap = gap;
            }
        }

        double[] row;
        if (partner != null) {
            double diff = bestX - getTagFieldOffsetInches(partner.getFiducialId());
            row = new double[]{
                    (p[0] - partnerPos[0]) / diff,
                    (p[1] - partnerPos[1]) / diff,
                    (p[2] - partnerPos[2]) / diff
            };
            tagsUsed = 2;
        } else {
            row = fieldDirectionInCamera(0);
            tagsUsed = 1;
        }

        return new double[]{
                p[0] - bestX * row[0],
                p[1] - bestX * row[1],
                p[2] - bestX * row[2]
        };
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

    private boolean isTargetTag(int id) {
        return id >= 38 && id <= 45;
    }

    private int clusterOf(int id) {
        return id <= 41 ? 0 : 1;
    }

    private double getTagFieldOffsetInches(int id) {
        double cm;
        switch (id) {
            case 38: cm = -17; break;
            case 39: cm = -7; break;
            case 40: cm = 7; break;
            case 41: cm = 17; break;
            case 45: cm = -17; break;
            case 44: cm = -7; break;
            case 43: cm = 7; break;
            case 42: cm = 17; break;
            default: cm = 0; break;
        }
        return cm / 2.54;
    }

    private void setDrive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }

    private void stopAll() {
        if (follower != null) {
            follower.breakFollowing();
        }
        setDrive(0, 0, 0);
        outtake.setPower(0);
        transfer.setPower(0);
        intake.setPower(0);
    }

    private double wrapDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private void initHardware() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.crservo.get("transfer");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setPower(0);
        transfer.setPower(0);
    }
}