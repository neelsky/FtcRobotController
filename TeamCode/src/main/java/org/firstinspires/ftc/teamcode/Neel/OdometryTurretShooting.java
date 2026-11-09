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
    public static double RELOCATE_MAX_SPEED_IN_S = 8;
    public static double RELOCATE_MAX_TURN_DEG_S = 30;

    private Follower follower;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Servo turret;
    private TagLocalizer localizer;
    private boolean located = false;
    private int fixes = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private Pose lastPose = null;
    private Alliance alliance = Alliance.BLUE;

    private int targetCell = 1;
    private double turretAngle = 0;
    private boolean turretCanReach = true;
    private double distanceToCell = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        follower = Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, false);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = hardwareMap.get(Servo.class, "turret");

        localizer = new TagLocalizer(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        while (opModeInInit()) {
            if (gamepad1.x) {
                alliance = Alliance.BLUE;
            }
            if (gamepad1.b) {
                alliance = Alliance.RED;
            }
            follower.updatePose();
            relocate(true);
            telemetry.addData("alliance", alliance + "   (X = blue, B = red)");
            telemetry.addData("position", located ? "FOUND from tags" : "not found yet: point the camera at 2 hive tags");
            telemetry.addData("tags", localizer.getLastResult());
            showPose();
            telemetry.update();
            turret.setPosition(TURRET_FORWARD_POS);
        }

        loopTimer.reset();
        while (opModeIsActive()) {
            follower.updatePose();
            relocate(false);

            if (located) {
                aimTurret();
            } else {
                turret.setPosition(TURRET_FORWARD_POS);
                turretCanReach = false;
            }

            boolean shooting = gamepad1.right_trigger > 0.1;
            if (shooting) {
                if (located) {
                    shooter.setGoalFromDistance(distanceToCell);
                } else {
                    shooter.useDefaultIfOff();
                }
            } else {
                shooter.setGoal(0);
            }
            shooter.update();
            if (gamepad1.left_trigger > 0.1) {
                shooter.forceFeed();
            } else {
                shooter.runFeeder(shooting && turretCanReach);
            }

            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            runIntakeButtons();

            if (gamepad1.right_bumper) {
                shooter.stop();
                intake.setIntake(0);
            }

            telemetry.addData("alliance", alliance);
            telemetry.addData("position", located ? "found (" + fixes + " tag fixes)" : "NOT FOUND: drive near the hive so the camera sees 2 tags");
            telemetry.addData("tags", localizer.getLastResult());
            showPose();
            telemetry.addData("aiming at", (blueFrameCellIsRear() ? "REAR" : "AUDIENCE") + " cell");
            telemetry.addData("distance", "%.1f in", distanceToCell);
            telemetry.addData("turret angle", "%.1f° %s", turretAngle, turretCanReach ? "" : "(OUT OF RANGE: turn the robot)");
            telemetry.addData("turret servo", "%.3f", turret.getPosition());
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }

        shooter.stop();
        intake.stop();
        localizer.stop();
    }

    private void relocate(boolean always) {
        Pose now = follower.getPose();
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();
        boolean still = true;
        if (lastPose != null && !always) {
            double speed = Math.hypot(now.getX() - lastPose.getX(), now.getY() - lastPose.getY()) / dt;
            double turn = Math.abs(wrapDegrees(Math.toDegrees(now.getHeading() - lastPose.getHeading()))) / dt;
            still = speed < RELOCATE_MAX_SPEED_IN_S && turn < RELOCATE_MAX_TURN_DEG_S;
        }
        lastPose = now;
        if (!still) {
            return;
        }
        Pose found = localizer.locate();
        if (found != null) {
            follower.setPose(found);
            lastPose = found;
            located = true;
            fixes++;
        }
    }

    private void aimTurret() {
        Pose p = blueFramePose();
        targetCell = p.getY() > PIVOT_Y ? 1 : 0;
        double aimY = targetCell == 1 ? PIVOT_Y + FieldMap.CELL_AIM_FROM_PIVOT_IN
                : PIVOT_Y - FieldMap.CELL_AIM_FROM_PIVOT_IN;

        double h = p.getHeading();
        double turretX = p.getX() + TURRET_FORWARD_OF_CENTER_IN * Math.cos(h) - TURRET_LEFT_OF_CENTER_IN * Math.sin(h);
        double turretY = p.getY() + TURRET_FORWARD_OF_CENTER_IN * Math.sin(h) + TURRET_LEFT_OF_CENTER_IN * Math.cos(h);

        double bearing = Math.toDegrees(Math.atan2(aimY - turretY, PIVOT_X - turretX));
        turretAngle = wrapDegrees(bearing - Math.toDegrees(h) + TURRET_TRIM_DEG);

        double units = turretAngle / TURRET_DEGREES_PER_UNIT;
        double position = TURRET_HIGHER_IS_LEFT ? TURRET_FORWARD_POS + units : TURRET_FORWARD_POS - units;
        turretCanReach = position >= 0 && position <= 1;
        turret.setPosition(Math.max(0, Math.min(1, position)));

        double tagsY = targetCell == 1 ? PIVOT_Y + FieldMap.UP_TAGS_FROM_PIVOT_IN
                : PIVOT_Y - FieldMap.UP_TAGS_FROM_PIVOT_IN;
        double flat = Math.hypot(PIVOT_X - p.getX(), tagsY - p.getY());
        double rise = FieldMap.UP_TAGS_HEIGHT_IN - Vision.CAMERA_HEIGHT_IN;
        distanceToCell = Math.hypot(flat, rise);
    }

    private boolean blueFrameCellIsRear() {
        return alliance == Alliance.RED ? targetCell == 0 : targetCell == 1;
    }

    private Pose blueFramePose() {
        Pose real = follower.getPose();
        if (alliance == Alliance.RED) {
            return new Pose(FIELD - real.getX(), FIELD - real.getY(), real.getHeading() - Math.PI);
        }
        return real;
    }

    private void showPose() {
        Pose p = follower.getPose();
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
