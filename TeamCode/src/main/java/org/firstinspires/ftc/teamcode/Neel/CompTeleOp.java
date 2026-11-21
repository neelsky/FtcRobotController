package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Vision;

public abstract class CompTeleOp extends LinearOpMode {
    protected abstract Alliance alliance();

    private Drivetrain drivetrain;
    private Shooter shooter;
    public static double MULTIPLIER_STEP = 0.02;
    public static double MULTIPLIER_MIN = 0.5;
    public static double MULTIPLIER_MAX = 1.5;
    private double shotMultiplier = 1.0;
    private boolean multUpWas = false;
    private boolean multDownWas = false;
    private boolean intakePulling = false;
    private Intake intake;
    private Vision vision;
    private final Aimer aimer = new Aimer();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        drivetrain = new Drivetrain(hardwareMap, true);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap, alliance());
        telemetry.addLine("Ready (" + alliance() + "). Press START.");
        telemetry.update();

        waitForStart();
        drivetrain.resetHeading();

        while (opModeIsActive()) {
            double heading = drivetrain.getHeading();
            boolean shooting = gamepad2.right_trigger > 0.1;

            vision.update(heading, -1);
            runShooter(shooting);
            drive(heading, shooting);
            runIntakeButtons();

            if (gamepad1.right_bumper) {
                shooter.stop();
                intake.setIntake(0);
                intakePulling = false;
            }
            if (intakePulling && !shooter.isFeeding() && gamepad2.left_trigger <= 0.1) {
                shooter.runTransferBackward();
            }

            vision.addTelemetry(telemetry);
            telemetry.addData("shot multiplier", "x%.2f   (gamepad 2: dpad up/down = %.2f steps, A = reset)", shotMultiplier, MULTIPLIER_STEP);
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }

        shooter.stop();
        intake.stop();
        vision.stop();
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

    private void runShooter(boolean shooting) {
        runMultiplierButtons();
        int fromDistance = vision.isTagVisible() ? Shooter.velocityForDistance(vision.getDistance()) : -1;
        if (!shooting) {
            shooter.setGoal(0);
        } else if (fromDistance > 0) {
            shooter.setGoal((int) Math.round(fromDistance * shotMultiplier));
        } else {
            shooter.useDefaultIfOff();
        }
        shooter.update();

        if (gamepad2.left_trigger > 0.1) {
            shooter.forceFeed();
        } else {
            shooter.runTriggerFeed(shooting, true);
        }
    }

    private void drive(double heading, boolean shooting) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double turnRate = drivetrain.getTurnRate();
        if (!shooting) {
            aimer.reset();
            aimer.noteTurnRate(turnRate);
        } else if (vision.isTagVisible()) {
            aimer.updateFromCamera(heading, turnRate, vision.getAngle());
        } else {
            aimer.noteTurnRate(turnRate);
        }

        if (shooting && aimer.hasAim()) {
            rx = aimer.turnPower(heading);
            telemetry.addData("aimError", "%.2f°", aimer.getError());
        }
        telemetry.addData("isAimedAtTarget", shooting && aimer.isAimed());

        drivetrain.drive(y, x, rx);
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

    }
}
