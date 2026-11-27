package org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer {
    public static double FEED_VELOCITY = -5600;
    public static double BACK_VELOCITY = 560;
    public static double PAUSE_BELOW = 0.90;
    public static double STUCK_READY_AT = 0.92;
    public static double DROP_FROM_PEAK = 0.04;
    public static double MIN_PAUSE_MS = 150;

    private final DcMotorEx transfer;
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private boolean feeding = false;
    private boolean paused = false;
    private String status = "waiting";
    private double peakVelocity = 0;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(0);
    }

    public void runTriggerFeed(Shooter shooter, boolean triggerHeld, boolean aimReady) {
        int goal = shooter.getGoal();
        double velocity = shooter.getVelocity();
        if (!triggerHeld || goal <= 0) {
            feeding = false;
            paused = false;
            peakVelocity = 0;
            status = "waiting";
        } else if (!feeding) {
            boolean ready = speedReady(shooter);
            if (ready && aimReady) {
                feeding = true;
                paused = false;
                peakVelocity = velocity;
            }
            status = !ready ? speedText(shooter) : aimReady ? "FEEDING" : "waiting for the turret to aim";
        } else if (paused) {
            if (speedReady(shooter) && pauseTimer.milliseconds() >= MIN_PAUSE_MS) {
                paused = false;
                peakVelocity = velocity;
            }
            status = paused ? "ball shot, " + speedText(shooter) : "FEEDING";
        } else {
            peakVelocity = Math.max(peakVelocity, velocity);
            if (velocity < peakVelocity * (1 - DROP_FROM_PEAK) || velocity < goal * PAUSE_BELOW) {
                paused = true;
                pauseTimer.reset();
                status = "ball shot, " + speedText(shooter);
            } else {
                status = "FEEDING";
            }
        }
        transfer.setVelocity(feeding && !paused ? FEED_VELOCITY : 0);
    }

    private boolean speedReady(Shooter shooter) {
        double velocity = shooter.getVelocity();
        int goal = shooter.getGoal();
        boolean maxedButClose = shooter.isMaxedOut() && velocity >= goal * STUCK_READY_AT;
        return goal > 0 && (shooter.atSpeed() || maxedButClose);
    }

    private String speedText(Shooter shooter) {
        return String.format("waiting for flywheel speed (%.0f of %d)%s", shooter.getVelocity(), shooter.getGoal(),
                shooter.isMaxedOut() ? " - MAXED OUT, charge the battery or lower that speed" : "");
    }

    public void startFeeding() {
        feeding = false;
        peakVelocity = 0;
        pauseTimer.reset();
    }

    public void runFeeder(Shooter shooter, boolean allowed) {
        double velocity = shooter.getVelocity();
        int goal = shooter.getGoal();
        if (!allowed || goal <= 0) {
            feeding = false;
            peakVelocity = 0;
        } else if (feeding) {
            peakVelocity = Math.max(peakVelocity, velocity);
            if (velocity < peakVelocity * (1 - DROP_FROM_PEAK)) {
                feeding = false;
                pauseTimer.reset();
            }
        } else {
            if (speedReady(shooter) && pauseTimer.milliseconds() >= MIN_PAUSE_MS) {
                feeding = true;
                peakVelocity = velocity;
            }
        }
        transfer.setVelocity(feeding ? FEED_VELOCITY : 0);
    }

    public void forceFeed() {
        feeding = false;
        transfer.setVelocity(FEED_VELOCITY);
    }

    public void runBackward() {
        feeding = false;
        transfer.setVelocity(BACK_VELOCITY);
    }

    public String getStatus() {
        return status;
    }

    public boolean isFeeding() {
        return feeding;
    }

    public void stop() {
        feeding = false;
        transfer.setVelocity(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("transfer", "%s | velocity %.0f", status, transfer.getVelocity());
    }
}
