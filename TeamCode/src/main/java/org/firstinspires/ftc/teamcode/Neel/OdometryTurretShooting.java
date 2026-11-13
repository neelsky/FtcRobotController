package org.firstinspires.ftc.teamcode.Neel;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Neel.subsystems.PoseKeeper;
import org.firstinspires.ftc.teamcode.Neel.subsystems.PoseMemory;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Neel.subsystems.TagLocalizer;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.FIELD;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.PIVOT_X;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.PIVOT_Y;
import static org.firstinspires.ftc.teamcode.Neel.subsystems.FieldMap.wrapDegrees;

@TeleOp(name = "Odometry Turret Shooting", group = "Glitch")
public class OdometryTurretShooting extends LinearOpMode {
    public static double TURRET_FORWARD_POS = 0.5;
    public static double TURRET_DEGREES_PER_UNIT = 270;
    public static boolean TURRET_HIGHER_IS_LEFT = true;
    public static double TURRET_FORWARD_OF_CENTER_IN = 0;
    public static double TURRET_LEFT_OF_CENTER_IN = 0;
    public static double TURRET_TRIM_DEG = 0;

    public static double BALL_SPEED_IN_PER_S = 120;
    public static double TURRET_LAG_S = 0.1;
    public static double VELOCITY_SMOOTHING = 0.3;
    public static double AUTO_POSE_MAX_AGE_S = 120;
    public static double SAVED_FILE_MAX_AGE_S = 120;
    public static double TURRET_SPEED_DEG_PER_S = 200;
    public static double TURRET_READY_DEG = 3;

    public static boolean USE_TAG_LOCATING = false;
    public static double RELOCATE_MAX_SPEED_IN_S = 8;
    public static double RELOCATE_MAX_TURN_DEG_S = 30;

    private Follower follower;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Servo turret;
    private TagLocalizer localizer;
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
    private boolean turretCanReach = false;
    private boolean turretReady = false;
    private double turretCommandPos = 0.5;
    private double turretEstimatePos = 0.5;
    private double distanceToCell = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        Pose kept = PoseKeeper.readPinpoint(hardwareMap);
        PoseKeeper.Saved saved = PoseKeeper.loadFile();
        boolean memoryFresh = PoseMemory.getPose() != null && PoseMemory.secondsSinceSave() < AUTO_POSE_MAX_AGE_S;
        if (memoryFresh) {
            alliance = PoseMemory.getAlliance();
        } else if (saved != null) {
            alliance = saved.alliance;
        }

        follower = Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, false);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = hardwareMap.get(Servo.class, "turret");
        if (USE_TAG_LOCATING) {
            localizer = new TagLocalizer(hardwareMap);
        }

        if (kept != null) {
            follower.setStartingPose(kept);
            located = true;
            positionSource = "kept by the Pinpoint";
        } else if (memoryFresh) {
            follower.setStartingPose(PoseMemory.getPose());
            located = true;
            positionSource = "carried over from the last OpMode";
        } else if (saved != null && saved.ageSeconds < SAVED_FILE_MAX_AGE_S) {
            follower.setStartingPose(saved.pose);
            located = true;
            positionSource = String.format("saved file (%.0f s old)", saved.ageSeconds);
        } else {
            follower.setStartingPose(new Pose(0, 0, 0));
        }

        while (opModeInInit()) {
            if (gamepad1.x) {
                alliance = Alliance.BLUE;
            }
            if (gamepad1.b) {
                alliance = Alliance.RED;
            }
            follower.updatePose();
            resyncButton();
            tagRelocate(true);
            telemetry.addData("alliance", alliance + "   (X = blue, B = red)");
            showPosition();
            telemetry.update();
            commandTurret(TURRET_FORWARD_POS, 0);
            turretEstimatePos = TURRET_FORWARD_POS;
        }

        loopTimer.reset();
        while (opModeIsActive()) {
            follower.updatePose();
            resyncButton();
            double dt = updateVelocity();
            tagRelocate(false);
            if (located) {
                PoseKeeper.save(follower.getPose(), alliance);
            }

            if (located) {
                aimTurret(dt);
            } else {
                commandTurret(TURRET_FORWARD_POS, dt);
                turretCanReach = false;
                turretReady = false;
            }

            boolean shooting = gamepad1.right_trigger > 0.1;
            if (!shooting) {
                shooter.setGoal(0);
            } else if (located) {
                shooter.setGoalFromDistance(distanceToCell);
            } else {
                shooter.useDefaultIfOff();
            }
            shooter.update();
            if (gamepad1.left_trigger > 0.1) {
                shooter.forceFeed();
            } else {
                shooter.runFeeder(shooting && turretCanReach && turretReady);
            }

            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            runIntakeButtons();

            if (gamepad1.right_bumper) {
                shooter.stop();
                intake.setIntake(0);
            }

            telemetry.addData("alliance", alliance);
            showPosition();
            telemetry.addData("aiming at", (blueFrameCellIsRear() ? "REAR" : "AUDIENCE") + " cell");
            telemetry.addData("distance", "%.1f in", distanceToCell);
            telemetry.addData("robot speed", "%.0f in/s", Math.hypot(vx, vy));
            telemetry.addData("turret angle", "%.1f° %s", turretAngle,
                    !turretCanReach ? "(CAN'T AIM: turn the robot)" : turretReady ? "ready" : "moving");
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }

        shooter.stop();
        intake.stop();
        if (localizer != null) {
            localizer.stop();
        }
    }

    private void resyncButton() {
        if (gamepad1.back) {
            follower.setPose(startSpot());
            lastBluePose = null;
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
        if (!still) {
            return;
        }
        Pose found = localizer.locate();
        if (found != null) {
            follower.setPose(found);
            lastBluePose = null;
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

    private void commandTurret(double position, double dt) {
        turretCommandPos = Math.max(0, Math.min(1, position));
        turret.setPosition(turretCommandPos);
        double maxStep = TURRET_SPEED_DEG_PER_S / TURRET_DEGREES_PER_UNIT * dt;
        double gap = turretCommandPos - turretEstimatePos;
        turretEstimatePos += Math.max(-maxStep, Math.min(maxStep, gap));
        turretReady = Math.abs(turretCommandPos - turretEstimatePos) * TURRET_DEGREES_PER_UNIT <= TURRET_READY_DEG;
    }

    private void aimTurret(double dt) {
        Pose p = blueFramePose();
        targetCell = p.getY() > PIVOT_Y ? 1 : 0;
        double aimY = targetCell == 1 ? PIVOT_Y + FieldMap.CELL_AIM_FROM_PIVOT_IN
                : PIVOT_Y - FieldMap.CELL_AIM_FROM_PIVOT_IN;
        double tagsY = targetCell == 1 ? PIVOT_Y + FieldMap.UP_TAGS_FROM_PIVOT_IN
                : PIVOT_Y - FieldMap.UP_TAGS_FROM_PIVOT_IN;

        double x = p.getX() + vx * TURRET_LAG_S;
        double y = p.getY() + vy * TURRET_LAG_S;
        double h = p.getHeading() + Math.toRadians(omegaDeg * TURRET_LAG_S);
        double turretX = x + TURRET_FORWARD_OF_CENTER_IN * Math.cos(h) - TURRET_LEFT_OF_CENTER_IN * Math.sin(h);
        double turretY = y + TURRET_FORWARD_OF_CENTER_IN * Math.sin(h) + TURRET_LEFT_OF_CENTER_IN * Math.cos(h);

        double flightTime = 0;
        double leadX = PIVOT_X;
        double leadY = aimY;
        for (int i = 0; i < 5; i++) {
            leadX = PIVOT_X - vx * flightTime;
            leadY = aimY - vy * flightTime;
            flightTime = Math.hypot(leadX - turretX, leadY - turretY) / BALL_SPEED_IN_PER_S;
        }

        double bearing = Math.toDegrees(Math.atan2(leadY - turretY, leadX - turretX));
        turretAngle = wrapDegrees(bearing - Math.toDegrees(h) + TURRET_TRIM_DEG);

        double units = turretAngle / TURRET_DEGREES_PER_UNIT;
        double position = TURRET_HIGHER_IS_LEFT ? TURRET_FORWARD_POS + units : TURRET_FORWARD_POS - units;
        turretCanReach = position >= 0 && position <= 1;
        commandTurret(position, dt);

        double flat = Math.hypot(PIVOT_X - vx * flightTime - x, tagsY - vy * flightTime - y);
        double rise = FieldMap.UP_TAGS_HEIGHT_IN - Vision.CAMERA_HEIGHT_IN;
        distanceToCell = Math.hypot(flat, rise);
    }

    private boolean blueFrameCellIsRear() {
        return alliance == Alliance.RED ? targetCell == 0 : targetCell == 1;
    }

    private Pose startSpot() {
        double r = CompAuto.ROBOT_HALF_IN;
        double x = FieldMap.CENTER + r + 1;
        double y = FIELD - r;
        if (alliance == Alliance.RED) {
            return new Pose(FIELD - x, FIELD - y, Math.toRadians(90));
        }
        return new Pose(x, y, Math.toRadians(-90));
    }

    private Pose blueFramePose() {
        Pose real = follower.getPose();
        if (alliance == Alliance.RED) {
            return new Pose(FIELD - real.getX(), FIELD - real.getY(), real.getHeading() - Math.PI);
        }
        return real;
    }

    private void showPosition() {
        Pose p = follower.getPose();
        telemetry.addData("position", located ? "known (" + positionSource + ")"
                : "UNKNOWN: put the robot on the auto start spot and press BACK/SHARE");
        telemetry.addData("robot", "x %.1f  y %.1f  h %.1f°", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    private void runIntakeButtons() {
        if (gamepad1.dpad_up) {
            intake.setIntake(Intake.OUT_POWER);
        }
        if (gamepad1.dpad_down) {
            intake.setIntake(Intake.IN_POWER);
        }
        if (gamepad1.dpad_left) {
            intake.setIntake(0);
        }
        if (gamepad1.a) {
            intake.setSides(Intake.SIDES_IN_POWER);
        }
        if (gamepad1.b) {
            intake.setSides(Intake.SIDES_OUT_POWER);
        }
        if (gamepad1.y) {
            intake.setSides(0);
        }
    }
}
