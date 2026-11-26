package org.firstinspires.ftc.teamcode.Neel;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;

@TeleOp(name = "Shooter Tuner", group = "Glitch")
public class ShooterTuner extends LinearOpMode {
    private static final String[] NAMES = {"TARGET", "P", "I", "D", "F"};
    private static final double[] STEPS = {50, 0.0002, 0.00005, 0.00002, 0.00002};

    private int selected = 0;
    private int target = 1200;
    private boolean running = false;
    private boolean aWas, upWas, downWas, leftWas, rightWas, xWas;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("SHOOTER TUNER");
        telemetry.addLine("A = flywheel on/off | dpad left/right = pick TARGET, P, I, D or F | dpad up/down = change it");
        telemetry.addLine("hold right bumper = 10x bigger steps | hold right trigger = feed | X = set the picked value to 0");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !aWas) {
                running = !running;
            }
            if (gamepad1.dpad_right && !rightWas) {
                selected = (selected + 1) % NAMES.length;
            }
            if (gamepad1.dpad_left && !leftWas) {
                selected = (selected + NAMES.length - 1) % NAMES.length;
            }
            double step = STEPS[selected] * (gamepad1.right_bumper ? 10 : 1);
            if (gamepad1.dpad_up && !upWas) {
                change(step);
            }
            if (gamepad1.dpad_down && !downWas) {
                change(-step);
            }
            if (gamepad1.x && !xWas) {
                setValue(0);
            }
            aWas = gamepad1.a;
            upWas = gamepad1.dpad_up;
            downWas = gamepad1.dpad_down;
            leftWas = gamepad1.dpad_left;
            rightWas = gamepad1.dpad_right;
            xWas = gamepad1.x;

            shooter.setGoal(running ? target : 0);
            shooter.update();
            boolean feeding = gamepad1.right_trigger > 0.1;
            if (feeding) {
                shooter.forceFeed();
                intake.setIntake(Intake.OUT_POWER);
            } else {
                shooter.stopFeeder();
                intake.setIntake(0);
            }

            double error = running ? target - shooter.getVelocity() : 0;
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData((i == selected ? ">> " : "   ") + NAMES[i], valueText(i));
            }
            telemetry.addData("flywheel", running ? "ON" : "off (press A)");
            telemetry.addData("velocity", "%.0f  (error %.0f)", shooter.getVelocity(), error);
            telemetry.addData("power", "%.3f", shooter.getPower());
            telemetry.addData("step", "%s per press%s", stepText(selected, step), gamepad1.right_bumper ? " (10x)" : "");
            telemetry.addLine("When it's good, copy P, I, D and F into the top of Shooter.java.");

            panels.addData("target", running ? target : 0);
            panels.addData("velocity", shooter.getVelocity());
            panels.addData("error", error);
            panels.addData("power", shooter.getPower());
            panels.addData("P", String.format("%.5f", Shooter.P));
            panels.addData("I", String.format("%.5f", Shooter.I));
            panels.addData("D", String.format("%.5f", Shooter.D));
            panels.addData("F", String.format("%.5f", Shooter.F));
            panels.update(telemetry);
        }
        shooter.stop();
        intake.stop();
    }

    private double getValue(int i) {
        switch (i) {
            case 0: return target;
            case 1: return Shooter.P;
            case 2: return Shooter.I;
            case 3: return Shooter.D;
            default: return Shooter.F;
        }
    }

    private void setValue(double value) {
        value = Math.max(0, value);
        switch (selected) {
            case 0: target = (int) Math.round(Math.min(value, Shooter.MAX_VELOCITY)); break;
            case 1: Shooter.P = value; break;
            case 2: Shooter.I = value; break;
            case 3: Shooter.D = value; break;
            default: Shooter.F = value; break;
        }
    }

    private void change(double amount) {
        setValue(getValue(selected) + amount);
    }

    private String valueText(int i) {
        return i == 0 ? String.valueOf(target) : String.format("%.5f", getValue(i));
    }

    private String stepText(int i, double step) {
        return i == 0 ? String.format("%.0f", step) : String.format("%.5f", step);
    }
}
