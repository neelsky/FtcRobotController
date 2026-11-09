package org.firstinspires.ftc.teamcode.Neel;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Neel.subsystems.PoseMemory;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.CENTER;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.FIELD;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.PIVOT_X;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.PIVOT_Y;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.UP_TAGS_FROM_PIVOT_IN;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.wrapDegrees;

public abstract class CompAuto extends LinearOpMode {
    protected abstract Alliance alliance();

    public static double ROBOT_HALF_IN = 9;
    public static double SHOOT_TAG_DISTANCE_IN = 36;
    public static double CHECK_TAG_DISTANCE_IN = 30;
    public static double CHECK_SIDE_SHIFT_IN = 2;
    public static double SHOOT_BACK_IN = 6;
    public static double BALLS_WALL_GAP_IN = 0.5;
    public static double BALLS_SWEEP_IN = 14;
    public static double BALLS_PUSH_IN = 34;
    public static double LANE_X = 113;
    public static double PARK_Y = 35.4;

    public static double DRIVE_MAX_POWER = 1.0;
    public static double INTAKE_MAX_POWER = 0.9;
    public static double PATH_TIMEOUT_S = 4;
    public static double SWEEP_TIMEOUT_S = 3.5;
    public static double INTAKE_EXTRA_S = 0.3;

    public static double FIRST_SHOOT_TIME_S = 7;
    public static double SECOND_SHOOT_TIME_S = 5.5;
    public static double AIM_TIMEOUT_S = 1.5;
    public static double AIM_SETTLE_MS = 250;
    public static double TILT_LOOK_TIME_S = 0.6;
    public static int TAG_SEEN_READINGS = 5;
    public static double TAG_FRESH_MS = 150;
    public static double PARK_START_S = 26;

    private static final int AUDIENCE = 0;
    private static final int REAR = 1;

    private Follower follower;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;
    private final Aimer aimer = new Aimer();

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime headingTimer = new ElapsedTime();
    private double lastHeading = 0;
    private double turnRate = 0;
    private int targetCell = -1;
    private int chosenCell = -1;
    private String seenTagIds = "none";
    private String step = "init";

    private double startX, startY, startH;
    private double shootRearX, shootRearY, shootRearH;
    private double shootAudienceX, shootAudienceY, shootAudienceH;
    private double turnX, turnY;
    private double ballsStartX, ballsEndX, ballsY;
    private double backOffX, backOffY;
    private double checkX, checkY, checkH;
    private double parkX;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        setUpPoses();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pose(startX, startY, startH));
        drivetrain = new Drivetrain(hardwareMap, false);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap, alliance());
        lastHeading = currentHeading();

        while (opModeInInit()) {
            updateHeading();
            vision.update(lastHeading, -1);
            Pose real = follower.getPose();
            telemetry.addData("alliance", alliance());
            telemetry.addData("start", "x %.1f  y %.1f  h %.0f°", real.getX(), real.getY(),
                    Math.toDegrees(real.getHeading()));
            vision.addTelemetry(telemetry);
            telemetry.addLine("Ready. Press START.");
            telemetry.update();
        }

        matchTimer.reset();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        step = "first shots";
        aimAndShoot(REAR, FIRST_SHOOT_TIME_S);

        step = "leaving wall";
        driveTo(pose(shootRearX, shootRearY, shootRearH), DRIVE_MAX_POWER, false, false);

        step = "to balls";
        intake.pickup(true);
        driveTo(pose(turnX, turnY, 0), DRIVE_MAX_POWER, true, false);
        driveTo(pose(ballsStartX, ballsY, 0), DRIVE_MAX_POWER, true, false);

        step = "intaking";
        driveTo(pose(ballsEndX, ballsY, 0), INTAKE_MAX_POWER, true, false, SWEEP_TIMEOUT_S);
        waitSeconds(INTAKE_EXTRA_S);

        step = "backing off";
        driveTo(pose(backOffX, backOffY, 0), DRIVE_MAX_POWER, true, false);
        intake.pickup(false);

        step = "checking tilt";
        driveTo(pose(checkX, checkY + 10, checkH), DRIVE_MAX_POWER, false, true);
        driveTo(pose(checkX, checkY, checkH), DRIVE_MAX_POWER, false, true);
        chosenCell = seesHiveTag(TILT_LOOK_TIME_S) ? REAR : AUDIENCE;

        step = "to " + cellName(chosenCell) + " cell";
        if (chosenCell == AUDIENCE) {
            driveTo(pose(checkX, checkY + 8, checkH), DRIVE_MAX_POWER, false, true);
            driveTo(pose(LANE_X, checkY + 8, checkH), DRIVE_MAX_POWER, false, true);
            driveTo(pose(LANE_X, shootAudienceY, shootAudienceH), DRIVE_MAX_POWER, false, true);
            driveTo(pose(shootAudienceX, shootAudienceY, shootAudienceH), DRIVE_MAX_POWER, false, true);
        } else {
            driveTo(pose(checkX, checkY + SHOOT_BACK_IN, checkH), DRIVE_MAX_POWER, false, true);
        }

        step = "shooting " + cellName(chosenCell) + " cell";
        aimAndShoot(chosenCell, SECOND_SHOOT_TIME_S);

        step = "parking";
        shooter.stop();
        intake.stop();
        if (chosenCell == AUDIENCE) {
            driveTo(pose(parkX, PARK_Y, shootAudienceH), DRIVE_MAX_POWER, false, false);
        } else {
            driveTo(pose(LANE_X, checkY + SHOOT_BACK_IN, checkH), DRIVE_MAX_POWER, false, false);
            driveTo(pose(LANE_X, PARK_Y, checkH), DRIVE_MAX_POWER, false, false);
            driveTo(pose(parkX, PARK_Y, checkH), DRIVE_MAX_POWER, false, false);
        }

        finish("parked");
    }

    private void setUpPoses() {
        double r = ROBOT_HALF_IN;
        double camFwd = Vision.CAMERA_FORWARD_OF_CENTER_IN;

        startX = CENTER + r + 1;
        startY = FIELD - r;
        startH = -90;

        shootRearX = PIVOT_X;
        shootRearY = PIVOT_Y + UP_TAGS_FROM_PIVOT_IN + SHOOT_TAG_DISTANCE_IN + camFwd;
        shootRearH = -90;
        shootAudienceX = PIVOT_X;
        shootAudienceY = PIVOT_Y - UP_TAGS_FROM_PIVOT_IN - SHOOT_TAG_DISTANCE_IN - camFwd;
        shootAudienceH = 90;

        turnX = 100;
        turnY = FIELD - r - 9;

        ballsY = FIELD - r - BALLS_WALL_GAP_IN;
        ballsStartX = FIELD - r - 0.5 - BALLS_SWEEP_IN;
        ballsEndX = FIELD - r - 0.5 + BALLS_PUSH_IN;
        backOffX = ballsStartX;
        backOffY = ballsY - 4;

        checkX = PIVOT_X - CHECK_SIDE_SHIFT_IN;
        checkY = PIVOT_Y + UP_TAGS_FROM_PIVOT_IN + CHECK_TAG_DISTANCE_IN + camFwd;
        checkH = -90;

        parkX = FIELD - r - 0.5;
    }

    private boolean isRed() {
        return alliance() == Alliance.RED;
    }

    private Pose pose(double x, double y, double headingDeg) {
        if (isRed()) {
            return new Pose(FIELD - x, FIELD - y, Math.toRadians(wrapDegrees(headingDeg + 180)));
        }
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    private Pose blueFramePose() {
        Pose real = follower.getPose();
        if (isRed()) {
            return new Pose(FIELD - real.getX(), FIELD - real.getY(), real.getHeading() - Math.PI);
        }
        return real;
    }

    private double currentHeading() {
        follower.update();
        PoseMemory.save(follower.getPose(), alliance());
        double deg = Math.toDegrees(follower.getPose().getHeading());
        return isRed() ? wrapDegrees(deg - 180) : deg;
    }

    private void updateHeading() {
        double heading = currentHeading();
        double dt = Math.max(headingTimer.seconds(), 1e-3);
        turnRate = Math.abs(wrapDegrees(heading - lastHeading)) / dt;
        lastHeading = heading;
        headingTimer.reset();
    }

    private String cellName(int cell) {
        boolean audience = cell == AUDIENCE;
        if (isRed()) {
            audience = !audience;
        }
        if (cell < 0) return "unknown";
        return audience ? "AUDIENCE" : "REAR";
    }

    private void driveTo(Pose target, double maxPower, boolean intakeOn, boolean preSpin) {
        driveTo(target, maxPower, intakeOn, preSpin, PATH_TIMEOUT_S);
    }

    private void driveTo(Pose target, double maxPower, boolean intakeOn, boolean preSpin, double timeoutS) {
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

        shooter.setGoal(preSpin ? Shooter.DEFAULT_VELOCITY : 0);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < timeoutS) {
            follower.update();
            PoseMemory.save(follower.getPose(), alliance());
            shooter.update();
            if (intakeOn) {
                intake.pickup(true);
            }
            Pose p = follower.getPose();
            telemetry.addData("step", step);
            telemetry.addData("pose", "x %.1f  y %.1f  h %.1f°", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            telemetry.update();
        }
        follower.breakFollowing();
        drivetrain.stop();
        lastHeading = currentHeading();
        headingTimer.reset();
    }

    private void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            shooter.update();
            telemetry.addData("step", step);
            telemetry.update();
        }
    }

    private boolean seesHiveTag(double seconds) {
        int readings = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            shooter.update();
            String ids = vision.freshHiveTagIds(TAG_FRESH_MS);
            if (!ids.isEmpty()) {
                readings++;
                seenTagIds = ids;
            }
            telemetry.addData("step", step);
            telemetry.addData("tag readings", "%d (need %d)", readings, TAG_SEEN_READINGS);
            telemetry.addData("tags seen", seenTagIds);
            telemetry.update();
            if (readings >= TAG_SEEN_READINGS) {
                return true;
            }
        }
        return false;
    }

    private void aimAndShoot(int cell, double shootSeconds) {
        targetCell = cell;
        aimer.reset();
        shooter.setGoal(0);

        ElapsedTime aimTimer = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        while (opModeIsActive() && aimTimer.seconds() < AIM_TIMEOUT_S) {
            updateHeading();
            vision.update(lastHeading, targetCell);
            setShooterGoal();
            shooter.update();
            turnTowardCell();
            if (!aimer.isAimed()) {
                settled.reset();
            } else if (settled.milliseconds() >= AIM_SETTLE_MS) {
                break;
            }
            telemetry.addData("step", step + " (aiming)");
            telemetry.update();
        }

        ElapsedTime shootTimer = new ElapsedTime();
        shooter.startFeeding();
        intake.setIntake(Intake.IN_POWER);
        while (opModeIsActive() && shootTimer.seconds() < shootSeconds && matchTimer.seconds() < PARK_START_S) {
            updateHeading();
            vision.update(lastHeading, targetCell);
            setShooterGoal();
            shooter.update();
            turnTowardCell();
            shooter.runFeeder(true);
            telemetry.addData("step", step);
            telemetry.addData("shooting", "%.1f / %.1f s", shootTimer.seconds(), shootSeconds);
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }

        shooter.stopFeeder();
        intake.setIntake(0);
        drivetrain.stop();
        targetCell = -1;
    }

    private void turnTowardCell() {
        if (vision.isTagVisible()) {
            aimer.updateFromCamera(lastHeading, turnRate, vision.getAngle());
            telemetry.addData("aimingAt", "CELL CENTER (camera)");
        } else if (targetCell >= 0) {
            aimer.noteTurnRate(turnRate);
            Pose p = blueFramePose();
            double aimY = targetCell == AUDIENCE ? PIVOT_Y - FieldMap.CELL_AIM_FROM_PIVOT_IN
                    : PIVOT_Y + FieldMap.CELL_AIM_FROM_PIVOT_IN;
            aimer.setAimHeading(Math.toDegrees(Math.atan2(aimY - p.getY(), PIVOT_X - p.getX())));
            telemetry.addData("aimingAt", "CELL CENTER (odometry, tag not in view)");
        }
        drivetrain.drive(0, 0, aimer.turnPower(lastHeading));
        telemetry.addData("aimError", "%.2f°", aimer.getError());
    }

    private void setShooterGoal() {
        if (vision.isTagVisible()) {
            shooter.setGoalFromDistance(vision.getDistance());
        } else if (targetCell >= 0) {
            Pose p = blueFramePose();
            double tagsY = targetCell == AUDIENCE ? PIVOT_Y - UP_TAGS_FROM_PIVOT_IN : PIVOT_Y + UP_TAGS_FROM_PIVOT_IN;
            double flat = Math.hypot(PIVOT_X - p.getX(), tagsY - p.getY());
            double rise = FieldMap.UP_TAGS_HEIGHT_IN - Vision.CAMERA_HEIGHT_IN;
            shooter.setGoalFromDistance(Math.hypot(flat, rise));
        } else {
            shooter.useDefaultIfOff();
        }
    }

    private void finish(String message) {
        step = message;
        follower.breakFollowing();
        drivetrain.stop();
        shooter.stop();
        intake.stop();
        vision.stop();
        telemetry.addData("step", step);
        telemetry.addData("cell shot", cellName(chosenCell));
        telemetry.addData("tags seen at check", seenTagIds);
        telemetry.update();
    }
}
