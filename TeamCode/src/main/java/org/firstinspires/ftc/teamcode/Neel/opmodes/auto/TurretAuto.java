package org.firstinspires.ftc.teamcode.Neel.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Neel.opmodes.teleops.OdometryTurretShooting;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Neel.subsystems.StatusLight;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.TurretAim;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.Vision;

import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.CENTER;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.FIELD;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.PIVOT_X;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.PIVOT_Y;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.UP_TAGS_FROM_PIVOT_IN;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap.wrapDegrees;

public abstract class TurretAuto extends LinearOpMode {
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
    public static double PARK_EDGE_SHIFT_IN = 8;
    public static double AUTO_INTAKE_POWER = -Intake.IN_POWER;
    public static double SHOOT_HEADING_DEG = 0;
    public static double START_OFF_WALL_IN = 4;

    public static double DRIVE_MAX_POWER = 1.0;
    public static double INTAKE_MAX_POWER = 0.9;
    public static double PATH_TIMEOUT_S = 4;
    public static double SWEEP_TIMEOUT_S = 3.5;
    public static double INTAKE_EXTRA_S = 0.3;

    public static double FIRST_SHOOT_TIME_S = 7;
    public static double SECOND_SHOOT_TIME_S = 5.5;
    public static double AIM_TIMEOUT_S = 2.0;
    public static double AIM_SETTLE_MS = 250;
    public static double TILT_LOOK_TIME_S = 0.8;
    public static int TAG_SEEN_READINGS = 5;
    public static double TAG_FRESH_MS = 150;
    public static double PARK_START_S = 26;

    private static final int AUDIENCE = 0;
    private static final int REAR = 1;

    private TurretAim aim;
    private Follower follower;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private StatusLight light;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private int chosenCell = -1;
    private String seenTagIds = "none";
    private String step = "init";

    private double startX, startY, startH;
    private double shootAudienceX, shootAudienceY;
    private double ballsStartX, ballsEndX, ballsY;
    private double backOffX, backOffY;
    private double checkX, checkY;
    private double parkX;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        setUpPoses();
        aim = new TurretAim(hardwareMap, alliance(), pose(startX, startY, startH));
        aim.setOdometryLocked(true);
        follower = aim.getFollower();
        drivetrain = new Drivetrain(hardwareMap, false);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        light = new StatusLight(hardwareMap);

        while (opModeInInit()) {
            aim.initLoop(false, false, false);
            telemetry.addLine("TURRET AUTO (" + alliance() + "). Put the robot on the normal auto start spot, then press START.");
            light.showAimed(false);
            aim.addInitTelemetry(telemetry);
            telemetry.addData("lights", light.status());
            telemetry.update();
        }

        matchTimer.reset();
        aim.start();
        if (isStopRequested()) {
            aim.stop();
            return;
        }

        double sh = SHOOT_HEADING_DEG;

        step = "turning to shoot";
        driveTo(pose(startX, startY - START_OFF_WALL_IN, sh), DRIVE_MAX_POWER, false, true);

        step = "first shots";
        aimAndShoot(FIRST_SHOOT_TIME_S);

        step = "to balls";
        intake.setIntake(AUTO_INTAKE_POWER);
        driveTo(pose(ballsStartX, ballsY, 0), DRIVE_MAX_POWER, true, false);

        step = "intaking";
        driveTo(pose(ballsEndX, ballsY, 0), INTAKE_MAX_POWER, true, false, SWEEP_TIMEOUT_S);
        waitSeconds(INTAKE_EXTRA_S);

        step = "backing off";
        driveTo(pose(backOffX, backOffY, 0), DRIVE_MAX_POWER, true, false);
        intake.setIntake(0);

        step = "checking tilt";
        driveTo(pose(checkX, checkY + 10, sh), DRIVE_MAX_POWER, false, true);
        driveTo(pose(checkX, checkY, sh), DRIVE_MAX_POWER, false, true);
        chosenCell = seesHiveTag(TILT_LOOK_TIME_S) ? REAR : AUDIENCE;

        step = "to " + cellName(chosenCell) + " cell";
        if (chosenCell == AUDIENCE) {
            driveTo(pose(checkX, checkY + 8, sh), DRIVE_MAX_POWER, false, true);
            driveTo(pose(LANE_X, checkY + 8, sh), DRIVE_MAX_POWER, false, true);
            driveTo(pose(LANE_X, shootAudienceY, sh), DRIVE_MAX_POWER, false, true);
            driveTo(pose(shootAudienceX, shootAudienceY, sh), DRIVE_MAX_POWER, false, true);
        } else {
            driveTo(pose(checkX, checkY + SHOOT_BACK_IN, sh), DRIVE_MAX_POWER, false, true);
        }

        step = "shooting " + cellName(chosenCell) + " cell";
        aimAndShoot(SECOND_SHOOT_TIME_S);

        step = "parking";
        shooter.stop();
        intake.stop();
        double parkY = PARK_Y - PARK_EDGE_SHIFT_IN;
        if (chosenCell == AUDIENCE) {
            driveTo(pose(LANE_X, parkY, sh), DRIVE_MAX_POWER, false, false);
            driveTo(pose(parkX, parkY, sh), DRIVE_MAX_POWER, false, false);
        } else {
            driveTo(pose(LANE_X, checkY + SHOOT_BACK_IN, sh), DRIVE_MAX_POWER, false, false);
            driveTo(pose(LANE_X, parkY, sh), DRIVE_MAX_POWER, false, false);
            driveTo(pose(parkX, parkY, sh), DRIVE_MAX_POWER, false, false);
        }

        finish("parked");
    }

    private void setUpPoses() {
        double r = ROBOT_HALF_IN;
        double camFwd = Vision.CAMERA_FORWARD_OF_CENTER_IN;

        startX = CENTER + r + 1;
        startY = FIELD - r;
        startH = -90;

        shootAudienceX = PIVOT_X;
        shootAudienceY = PIVOT_Y - UP_TAGS_FROM_PIVOT_IN - SHOOT_TAG_DISTANCE_IN - camFwd;

        ballsY = FIELD - r - BALLS_WALL_GAP_IN;
        ballsStartX = FIELD - r - 0.5 - BALLS_SWEEP_IN;
        ballsEndX = FIELD - r - 0.5 + BALLS_PUSH_IN;
        backOffX = ballsStartX;
        backOffY = ballsY - 4;

        checkX = PIVOT_X - CHECK_SIDE_SHIFT_IN;
        checkY = PIVOT_Y + UP_TAGS_FROM_PIVOT_IN + CHECK_TAG_DISTANCE_IN + camFwd;

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

    private String cellName(int cell) {
        if (cell < 0) {
            return "unknown";
        }
        boolean audience = cell == AUDIENCE;
        if (isRed()) {
            audience = !audience;
        }
        return audience ? "AUDIENCE" : "REAR";
    }

    private void setShotGoal() {
        double velocity = aim.isAimKnown() ? OdometryTurretShooting.shotVelocityFor(aim.getDistanceToCell()) : -1;
        if (velocity > 0) {
            shooter.setGoal((int) Math.round(velocity));
        } else {
            shooter.useDefaultIfOff();
        }
    }

    private void updateLight() {
        light.showAimed(aim.isAimed());
    }

    private void showTelemetry() {
        telemetry.addData("step", step);
        telemetry.addData("match time", "%.1f s", matchTimer.seconds());
        aim.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.update();
    }

    private void driveTo(Pose target, double maxPower, boolean intakeOn, boolean preSpin) {
        driveTo(target, maxPower, intakeOn, preSpin, PATH_TIMEOUT_S);
    }

    private void driveTo(Pose target, double maxPower, boolean intakeOn, boolean preSpin, double timeoutS) {
        follower.updatePose();
        Pose from = follower.getPose();
        double endHeading = from.getHeading()
                + Math.toRadians(wrapDegrees(Math.toDegrees(target.getHeading() - from.getHeading())));
        target = new Pose(target.getX(), target.getY(), endHeading);
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(from, target))
                .setLinearHeadingInterpolation(from.getHeading(), target.getHeading())
                .build();
        follower.followPath(path, maxPower, true);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < timeoutS) {
            follower.update();
            aim.updateWhileFollowing(false);
            if (preSpin) {
                setShotGoal();
            } else {
                shooter.setGoal(0);
            }
            shooter.update();
            shooter.stopFeeder();
            if (intakeOn) {
                intake.setIntake(AUTO_INTAKE_POWER);
            }
            updateLight();
            showTelemetry();
        }
        follower.breakFollowing();
        drivetrain.stop();
    }

    private void holdStill(boolean shooting) {
        aim.update(false, 0, shooting, false);
        updateLight();
    }

    private void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            holdStill(false);
            shooter.update();
            showTelemetry();
        }
    }

    private boolean seesHiveTag(double seconds) {
        Vision vision = aim.getVision();
        if (vision == null) {
            return false;
        }
        int readings = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            holdStill(false);
            setShotGoal();
            shooter.update();
            String ids = vision.freshHiveTagIds(TAG_FRESH_MS);
            if (!ids.isEmpty()) {
                readings++;
                seenTagIds = ids;
            }
            telemetry.addData("tag readings", "%d (need %d)", readings, TAG_SEEN_READINGS);
            telemetry.addData("tags seen", seenTagIds);
            showTelemetry();
            if (readings >= TAG_SEEN_READINGS) {
                return true;
            }
        }
        return false;
    }

    private void aimAndShoot(double shootSeconds) {
        ElapsedTime aimTimer = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        boolean settledInTime = false;
        while (opModeIsActive() && aimTimer.seconds() < AIM_TIMEOUT_S) {
            holdStill(true);
            setShotGoal();
            shooter.update();
            if (!aim.isAimed()) {
                settled.reset();
            } else if (settled.milliseconds() >= AIM_SETTLE_MS) {
                settledInTime = true;
                break;
            }
            telemetry.addData("shooting", "aiming the turret");
            showTelemetry();
        }

        ElapsedTime shootTimer = new ElapsedTime();
        shooter.startFeeding();
        intake.setIntake(AUTO_INTAKE_POWER);
        while (opModeIsActive() && shootTimer.seconds() < shootSeconds && matchTimer.seconds() < PARK_START_S) {
            holdStill(true);
            setShotGoal();
            shooter.update();
            boolean aimOk = settledInTime ? aim.isAimed()
                    : aim.isAimKnown() && aim.canReach() && aim.isTurretReady();
            shooter.runFeeder(aimOk);
            telemetry.addData("shooting", "%.1f / %.1f s%s", shootTimer.seconds(), shootSeconds, aimOk ? "" : " (waiting for the turret)");
            showTelemetry();
        }

        shooter.stopFeeder();
        intake.setIntake(0);
    }

    private void finish(String message) {
        step = message;
        follower.breakFollowing();
        drivetrain.stop();
        shooter.stop();
        intake.stop();
        aim.stop();
        light.showAimed(false);
        telemetry.addData("step", step);
        telemetry.addData("cell shot", cellName(chosenCell));
        telemetry.addData("tags seen at check", seenTagIds);
        telemetry.update();
    }
}
