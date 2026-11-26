package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Neel.subsystems.StatusLight;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.TurretAim;

@TeleOp(name = "Odometry Turret Shooting", group = "Glitch")
public class OdometryTurretShooting extends LinearOpMode {
    public static double TURRET_ZERO_POS = TurretAim.TURRET_ZERO_POS;
    public static double TURRET_LIMIT_DEG = TurretAim.TURRET_LIMIT_DEG;
    public static double TURRET_DEGREES_PER_UNIT = TurretAim.TURRET_DEGREES_PER_UNIT;
    public static boolean TURRET_HIGHER_IS_LEFT = TurretAim.TURRET_HIGHER_IS_LEFT;

    public static double MULTIPLIER_STEP = 0.02;
    public static double MULTIPLIER_MIN = 0.5;
    public static double MULTIPLIER_MAX = 1.5;

    private TurretAim aim;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private StatusLight light;
    private double shotMultiplier = 1.0;
    private boolean multUpWas = false;
    private boolean multDownWas = false;
    private boolean intakePulling = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        aim = new TurretAim(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, false);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        light = new StatusLight(hardwareMap);

        while (opModeInInit()) {
            aim.initLoop(gamepad1.x, gamepad1.b, gamepad1.back);
            TURRET_HIGHER_IS_LEFT = TurretAim.TURRET_HIGHER_IS_LEFT;
            light.showAimed(false);
            aim.addInitTelemetry(telemetry);
            telemetry.addData("lights", light.status());
            telemetry.update();
        }

        aim.start();
        while (opModeIsActive()) {
            boolean shooting = gamepad2.right_trigger > 0.1;
            aim.update(gamepad1.back, gamepad1.right_stick_x, shooting, gamepad2.y);
            TURRET_HIGHER_IS_LEFT = TurretAim.TURRET_HIGHER_IS_LEFT;

            runMultiplierButtons();
            if (!shooting) {
                shooter.setGoal(0);
            } else if (aim.isAimKnown()) {
                setShotGoal(aim.getDistanceToCell());
            } else {
                shooter.useDefaultIfOff();
            }
            shooter.update();
            if (gamepad2.left_trigger > 0.1) {
                shooter.forceFeed();
            } else {
                shooter.runTriggerFeed(shooting, aim.isAimed());
            }

            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            runIntakeButtons();

            if (gamepad1.right_bumper) {
                shooter.stop();
                intake.setIntake(0);
                intakePulling = false;
            }
            if (intakePulling && !shooter.isFeeding() && gamepad2.left_trigger <= 0.1) {
                shooter.runTransferBackward();
            }

            updateLight();
            telemetry.addData("READY TO SHOOT", !aim.isAimKnown() ? "NO - looking for the tags"
                    : !aim.canReach() ? "NO - turret at its limit"
                    : !aim.isTurretReady() ? "NO - turret turning"
                    : !aim.cameraAgrees() ? "NO - camera says the turret isn't there yet"
                    : shooting && !flywheelAtSpeed() ? "AIMED - flywheel spinning up"
                    : "YES");
            aim.addTelemetry(telemetry);
            telemetry.addData("lights", light.status());
            telemetry.addData("flower intake", intake.hasFlower() ? "gamepad 2 dpad right / left" : "NOT FOUND: add a servo named flowerIntake to the config");
            telemetry.addData("shot multiplier", "x%.2f   (gamepad 2: dpad up/down = %.2f steps, A = reset)", shotMultiplier, MULTIPLIER_STEP);
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }

        shooter.stop();
        intake.stop();
        aim.stop();
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
            shotMultiplier = Math.min(MULTIPLIER_MAX, shotMultiplier + MULTIPLIER_STEP);
        }
        if (gamepad2.dpad_down && !multDownWas) {
            shotMultiplier = Math.max(MULTIPLIER_MIN, shotMultiplier - MULTIPLIER_STEP);
        }
        if (gamepad2.a) {
            shotMultiplier = 1.0;
        }
        multUpWas = gamepad2.dpad_up;
        multDownWas = gamepad2.dpad_down;
    }

    public static double shotVelocityFor(double distanceIn) {
        return Shooter.velocityForDistance(distanceIn);
    }

    private void setShotGoal(double distanceIn) {
        double velocity = shotVelocityFor(distanceIn);
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
