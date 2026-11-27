package org.firstinspires.ftc.teamcode.Kekoa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Neel.opmodes.teleops.OdometryTurretShooting;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.Intake;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.Shooter;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.Transfer;
import org.firstinspires.ftc.teamcode.Neel.subsystems.StatusLight;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.Turret;

@TeleOp(name = "Kekoa Drive (Kekoa folder)", group = "Glitch")
public class KekoaDrive extends LinearOpMode {
    public static double STRAFE_BOOST = 1.1;
    public static boolean ALWAYS_REV = true;

    private Turret aim;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Transfer transfer;
    private Intake intake;
    private StatusLight light;
    private double shotMultiplier = 1.0;
    private boolean multUpWas = false;
    private boolean multDownWas = false;
    private boolean intakePulling = false;
    private double forwardHeadingRad = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        aim = new Turret(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, false);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        light = new StatusLight(hardwareMap);

        while (opModeInInit()) {
            aim.initLoop(gamepad1.x, gamepad1.b, gamepad1.back);
            OdometryTurretShooting.TURRET_HIGHER_IS_LEFT = Turret.TURRET_HIGHER_IS_LEFT;
            light.showAimed(false);
            telemetry.addLine("KEKOA DRIVE: field oriented. Gamepad 1 Y = forward is the way the robot faces now.");
            aim.addInitTelemetry(telemetry);
            telemetry.addData("lights", light.status());
            telemetry.update();
        }

        aim.start();
        aim.getFollower().updatePose();
        forwardHeadingRad = aim.getFieldHeadingRad();
        while (opModeIsActive()) {
            boolean shooting = gamepad2.right_trigger > 0.1;
            aim.update(gamepad1.back, gamepad1.right_stick_x, shooting, gamepad2.y);
            OdometryTurretShooting.TURRET_HIGHER_IS_LEFT = Turret.TURRET_HIGHER_IS_LEFT;

            runMultiplierButtons();
            if (!shooting && !ALWAYS_REV) {
                shooter.setGoal(0);
            } else if (aim.isAimKnown()) {
                setShotGoal(aim.getDistanceToCell());
            } else {
                shooter.useDefaultIfOff();
            }
            shooter.update();
            if (gamepad2.left_trigger > 0.1) {
                transfer.forceFeed();
            } else {
                transfer.runTriggerFeed(shooter, shooting, aim.isAimed());
            }

            if (gamepad1.y) {
                forwardHeadingRad = aim.getFieldHeadingRad();
            }
            driveFieldOriented();
            runIntakeButtons();

            if (gamepad1.right_bumper) {
                if (!ALWAYS_REV) {
                    shooter.stop();
                    transfer.stop();
                }
                intake.setIntake(0);
                intakePulling = false;
            }
            if (intakePulling && !transfer.isFeeding() && gamepad2.left_trigger <= 0.1) {
                transfer.runBackward();
            }

            updateLight();
            telemetry.addData("READY TO SHOOT", !aim.isAimKnown() ? "NO - looking for the tags"
                    : !aim.canReach() ? "NO - turret at its limit"
                    : !aim.isTurretReady() ? "NO - turret turning"
                    : !aim.cameraAgrees() ? "NO - camera says the turret isn't there yet"
                    : !flywheelAtSpeed() ? "AIMED - flywheel spinning up"
                    : "YES");
            telemetry.addData("driving", "field oriented (gamepad 1 Y = reset forward) | robot %.0f° from forward",
                    Math.toDegrees(wrap(aim.getFieldHeadingRad() - forwardHeadingRad)));
            telemetry.addData("flywheel", ALWAYS_REV ? "always revved up" : "spins up when you shoot");
            aim.addTelemetry(telemetry);
            telemetry.addData("lights", light.status());
            telemetry.addData("flower intake", intake.hasFlower() ? "gamepad 2 dpad right / left" : "NOT FOUND: add a servo named flowerIntake to the config");
            telemetry.addData("shot multiplier", "x%.2f   (gamepad 2: dpad up/down = %.2f steps, A = reset)", shotMultiplier, OdometryTurretShooting.MULTIPLIER_STEP);
            shooter.addTelemetry(telemetry);
            transfer.addTelemetry(telemetry);
            telemetry.update();
        }

        shooter.stop();
        transfer.stop();
        intake.stop();
        aim.stop();
    }

    private void driveFieldOriented() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double theta = aim.getFieldHeadingRad() - forwardHeadingRad;
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double robotForward = forward * cos - right * sin;
        double robotRight = forward * sin + right * cos;
        drivetrain.drive(robotForward, robotRight * STRAFE_BOOST, turn);
    }

    private static double wrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private void updateLight() {
        light.showAimed(aim.isAimed());
    }

    private boolean flywheelAtSpeed() {
        double goal = shooter.getGoal();
        double velocity = Math.abs(shooter.getVelocity());
        return goal > 0 && velocity >= goal * Shooter.READY_AT && velocity <= goal * (2 - Shooter.READY_AT);
    }

    private void runMultiplierButtons() {
        if (gamepad2.dpad_up && !multUpWas) {
            shotMultiplier = Math.min(OdometryTurretShooting.MULTIPLIER_MAX, shotMultiplier + OdometryTurretShooting.MULTIPLIER_STEP);
        }
        if (gamepad2.dpad_down && !multDownWas) {
            shotMultiplier = Math.max(OdometryTurretShooting.MULTIPLIER_MIN, shotMultiplier - OdometryTurretShooting.MULTIPLIER_STEP);
        }
        if (gamepad2.a) {
            shotMultiplier = 1.0;
        }
        multUpWas = gamepad2.dpad_up;
        multDownWas = gamepad2.dpad_down;
    }

    private void setShotGoal(double distanceIn) {
        double velocity = OdometryTurretShooting.shotVelocityFor(distanceIn);
        if (velocity <= 0) {
            shooter.useDefaultIfOff();
            return;
        }
        shooter.setGoal((int) Math.round(velocity * shotMultiplier));
    }

    private void runIntakeButtons() {
        if (gamepad1.dpad_up) {
            intake.setIntake(Intake.IN_POWER);
            intakePulling = false;
        }
        if (gamepad1.dpad_down) {
            intake.setIntake(Intake.OUT_POWER);
            intakePulling = true;
        }
        if (gamepad1.dpad_left) {
            intake.setIntake(0);
            intakePulling = false;
        }
        if (gamepad2.dpad_right) {
            intake.flowerRight();
        }
        if (gamepad2.dpad_left) {
            intake.flowerLeft();
        }
    }
}
