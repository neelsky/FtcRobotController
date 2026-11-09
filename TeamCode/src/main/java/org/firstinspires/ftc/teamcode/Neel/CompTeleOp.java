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
            boolean shooting = gamepad1.right_trigger > 0.1;

            vision.update(heading, -1);
            runShooter(shooting);
            drive(heading, shooting);
            runIntakeButtons();

            if (gamepad1.right_bumper) {
                shooter.stop();
                intake.setIntake(0);
            }

            vision.addTelemetry(telemetry);
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }

        shooter.stop();
        intake.stop();
        vision.stop();
    }

    private void runShooter(boolean shooting) {
        if (!shooting) {
            shooter.setGoal(0);
        } else if (vision.isTagVisible()) {
            shooter.setGoalFromDistance(vision.getDistance());
        } else {
            shooter.useDefaultIfOff();
        }
        shooter.update();

        if (gamepad1.left_trigger > 0.1) {
            shooter.forceFeed();
        } else {
            shooter.runFeeder(shooting);
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
