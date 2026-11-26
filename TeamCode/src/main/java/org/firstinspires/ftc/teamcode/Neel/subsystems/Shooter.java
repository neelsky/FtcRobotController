package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    public static double P = 0.0052;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.00072;
    public static double MAX_I_POWER = 0.2;
    public static double RESET_I_ON_GOAL_CHANGE = 100;
    public static double FORMULA_SLOPE = 4.086;
    public static double FORMULA_OFFSET = 666;
    public static double FORMULA_SCALE = 0.62;
    public static int DEFAULT_VELOCITY = 600;
    public static int MAX_VELOCITY = 2200;
    public static double MIN_DISTANCE_IN = 32;
    public static double MAX_DISTANCE_IN = 114;

    public static double READY_AT = 0.97;
    public static double DROP_FROM_PEAK = 0.03;
    public static double MIN_PAUSE_MS = 150;
    public static double MAX_PAUSE_MS = 500;
    public static double SPIN_UP_MS = 1200;
    public static double TRANSFER_VELOCITY = -5600;
    public static double TRANSFER_BACK_VELOCITY = 560;

    private final DcMotorEx outtake;
    private final DcMotorEx transfer;
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private int goalVelocity = 0;
    private double velocity = 0;
    private double power = 0;
    private double peakVelocity = 0;
    private boolean feeding = false;
    private double integral = 0;
    private double lastError = 0;
    private int lastGoal = 0;

    public Shooter(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setPower(0);
        transfer.setPower(0);
    }

    public static int velocityForDistance(double distanceIn) {
        if (Double.isNaN(distanceIn) || distanceIn <= 0) {
            return -1;
        }
        double d = Math.max(MIN_DISTANCE_IN, Math.min(MAX_DISTANCE_IN, distanceIn));
        return Math.min((int) Math.round((FORMULA_SLOPE * d + FORMULA_OFFSET) * FORMULA_SCALE), MAX_VELOCITY);
    }

    public void setGoal(int velocity) {
        goalVelocity = Math.max(0, Math.min(velocity, MAX_VELOCITY));
    }

    public void setGoalFromDistance(double distanceIn) {
        int fromDistance = velocityForDistance(distanceIn);
        if (fromDistance > 0) {
            setGoal(fromDistance);
        } else if (goalVelocity == 0) {
            setGoal(DEFAULT_VELOCITY);
        }
    }

    public void useDefaultIfOff() {
        if (goalVelocity == 0) {
            setGoal(DEFAULT_VELOCITY);
        }
    }

    public int getGoal() {
        return goalVelocity;
    }

    public double getVelocity() {
        return velocity;
    }

    public void update() {
        velocity = Math.abs(outtake.getVelocity());
        double dt = Math.max(1e-3, Math.min(0.1, loopTimer.seconds()));
        loopTimer.reset();
        power = 0;
        if (goalVelocity <= 0 || Math.abs(goalVelocity - lastGoal) > RESET_I_ON_GOAL_CHANGE) {
            integral = 0;
            lastError = goalVelocity - velocity;
        }
        lastGoal = goalVelocity;
        if (goalVelocity > 0) {
            double error = goalVelocity - velocity;
            integral += error * dt;
            if (I > 0) {
                double maxIntegral = MAX_I_POWER / I;
                integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
            }
            double derivative = (error - lastError) / dt;
            lastError = error;
            power = F * goalVelocity + P * error + I * integral + D * derivative;
            power = Math.max(0, Math.min(1, power));
        }
        outtake.setPower(power);
    }

    public double getPower() {
        return power;
    }

    public void startFeeding() {
        feeding = false;
        peakVelocity = 0;
        spinUpTimer.reset();
        pauseTimer.reset();
    }

    public void runFeeder(boolean allowed) {
        if (!allowed || goalVelocity <= 0) {
            feeding = false;
            peakVelocity = 0;
            spinUpTimer.reset();
        } else if (feeding) {
            peakVelocity = Math.max(peakVelocity, velocity);
            if (velocity < peakVelocity * (1 - DROP_FROM_PEAK)) {
                feeding = false;
                pauseTimer.reset();
            }
        } else {
            boolean atSpeed = velocity >= goalVelocity * READY_AT && velocity <= goalVelocity * (2 - READY_AT);
            boolean spunUp = spinUpTimer.milliseconds() >= SPIN_UP_MS && velocity < goalVelocity;
            boolean pausedLongEnough = pauseTimer.milliseconds() >= MAX_PAUSE_MS;
            if ((atSpeed || (spunUp && pausedLongEnough)) && pauseTimer.milliseconds() >= MIN_PAUSE_MS) {
                feeding = true;
                peakVelocity = velocity;
            }
        }
        transfer.setVelocity(feeding ? TRANSFER_VELOCITY : 0);
    }

    public void runTriggerFeed(boolean triggerHeld, boolean aimReady) {
        if (!triggerHeld || goalVelocity <= 0) {
            feeding = false;
            spinUpTimer.reset();
        } else if (!feeding) {
            boolean atSpeed = velocity >= goalVelocity * READY_AT;
            boolean spunUp = spinUpTimer.milliseconds() >= SPIN_UP_MS;
            if ((atSpeed || spunUp) && aimReady) {
                feeding = true;
            }
        }
        transfer.setVelocity(feeding ? TRANSFER_VELOCITY : 0);
    }

    public void forceFeed() {
        feeding = false;
        transfer.setVelocity(TRANSFER_VELOCITY);
    }

    public void runTransferBackward() {
        feeding = false;
        transfer.setVelocity(TRANSFER_BACK_VELOCITY);
    }

    public boolean isFeeding() {
        return feeding;
    }

    public void stopFeeder() {
        feeding = false;
        transfer.setVelocity(0);
    }

    public void stop() {
        goalVelocity = 0;
        stopFeeder();
        power = 0;
        outtake.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("goalVelocity", goalVelocity);
        telemetry.addData("curVelocity", "%.0f", velocity);
        telemetry.addData("outtakePower", "%.2f", power);
        telemetry.addData("transfer", feeding ? "FEEDING" : "waiting");
        telemetry.addData("transferVelocity", "%.0f", transfer.getVelocity());
    }
}
