package org.firstinspires.ftc.teamcode.Neel;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter Tuner", group = "Glitch")
public class ShooterTuner extends LinearOpMode {
    private double TARGET_VELOCITY = 700;
    private double P = 0.0;
    private double I = 0.0;
    private double D = 0.0;
    private double F = 0.00117;
    public static double TRANSFER_POWER = 1.0;
    public static double INTAKE_POWER = -0.5;

    private double integral = 0;
    private double lastError = 0;
    private boolean running = false;
    private boolean aWasDown = false;
    private boolean upWasDown = false;
    private boolean downWasDown = false;
    private static final double F_STEP = 0.00005;
    private static final double P_STEP = 0.0002;
    private boolean leftWasDown = false;
    private boolean rightWasDown = false;
    private boolean yWasDown = false;

    @Override
    public void runOpMode() {
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        CRServo transfer = hardwareMap.crservo.get("transfer");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("A = flywheel on/off. Hold right trigger = feed. Dpad up/down = F. Bumpers or dpad left/right = P. Y = target 500/600/700.");
        telemetry.update();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            if (gamepad1.a && !aWasDown) {
                running = !running;
                integral = 0;
                lastError = 0;
            }
            aWasDown = gamepad1.a;
            if (gamepad1.dpad_up && !upWasDown) {
                F += F_STEP;
            }
            if (gamepad1.dpad_down && !downWasDown) {
                F = Math.max(0, F - F_STEP);
            }
            boolean pUp = gamepad1.dpad_right || gamepad1.right_bumper;
            boolean pDown = gamepad1.dpad_left || gamepad1.left_bumper;
            if (pUp && !rightWasDown) {
                P += P_STEP;
            }
            if (pDown && !leftWasDown) {
                P = Math.max(0, P - P_STEP);
            }
            if (gamepad1.y && !yWasDown) {
                TARGET_VELOCITY = TARGET_VELOCITY >= 700 ? 500 : TARGET_VELOCITY + 100;
            }
            rightWasDown = pUp;
            leftWasDown = pDown;
            yWasDown = gamepad1.y;
            upWasDown = gamepad1.dpad_up;
            downWasDown = gamepad1.dpad_down;

            double velocity = Math.abs(outtake.getVelocity());
            double dt = Math.max(timer.seconds(), 1e-3);
            timer.reset();

            double power = 0;
            double error = 0;
            if (running) {
                error = TARGET_VELOCITY - velocity;
                integral += error * dt;
                double derivative = (error - lastError) / dt;
                lastError = error;
                power = F * TARGET_VELOCITY + P * error + I * integral + D * derivative;
                power = Math.max(0, Math.min(1, power));
            }
            outtake.setPower(power);

            boolean feeding = gamepad1.right_trigger > 0.1;
            transfer.setPower(feeding ? TRANSFER_POWER : 0);
            intake.setPower(feeding ? INTAKE_POWER : 0);

            panels.addData("version", "v3");
            panels.addData("buttons", (gamepad1.left_bumper ? "LB " : "") + (gamepad1.right_bumper ? "RB " : "")
                    + (gamepad1.dpad_left ? "LEFT " : "") + (gamepad1.dpad_right ? "RIGHT " : "")
                    + (gamepad1.dpad_up ? "UP " : "") + (gamepad1.dpad_down ? "DOWN " : ""));
            panels.addData("target", running ? TARGET_VELOCITY : 0);
            panels.addData("velocity", velocity);
            panels.addData("error", error);
            panels.addData("power", power);
            panels.addData("F", String.format("%.5f", F));
            panels.addData("P", String.format("%.4f", P));
            panels.update(telemetry);
        }

        outtake.setPower(0);
        transfer.setPower(0);
        intake.setPower(0);
    }
}