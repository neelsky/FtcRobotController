package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    public static double F = 0.00117;
    public static double P = 0.0042;
    public static int DEFAULT_VELOCITY = 900;
    public static int MAX_VELOCITY = 2200;
    public static double MIN_DISTANCE_IN = 32;
    public static double MAX_DISTANCE_IN = 114;

    public static double READY_AT = 0.97;
    public static double DROP_FROM_PEAK = 0.03;
    public static double MIN_PAUSE_MS = 150;
    public static double MAX_PAUSE_MS = 500;
    public static double SPIN_UP_MS = 1200;
    public static double TRANSFER_POWER = 1.0;

    private final DcMotorEx outtake;
    private final CRServo transfer;
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private final ElapsedTime fullPowerTimer = new ElapsedTime();

    private int goalVelocity = 0;
    private double velocity = 0;
    private double peakVelocity = 0;
    private double power = 0;
    private boolean feeding = false;

    public Shooter(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        transfer = hardwareMap.crservo.get("transfer");
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setPower(0);
        transfer.setPower(0);
    }

    public static int velocityForDistance(double distanceIn) {
        if (distanceIn <= MIN_DISTANCE_IN || distanceIn >= MAX_DISTANCE_IN) {
            return -1;
        }
        return Math.min((int) Math.round((4.086 * distanceIn + 666) * 0.62), MAX_VELOCITY);
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
        power = 0;
        if (goalVelocity > 0) {
            power = F * goalVelocity + P * (goalVelocity - velocity);
            power = Math.max(0, Math.min(1, power));
        }
        outtake.setPower(power);
        if (power < 1) {
            fullPowerTimer.reset();
        }
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
            boolean atSpeed = velocity >= goalVelocity * READY_AT;
            boolean spunUp = spinUpTimer.milliseconds() >= SPIN_UP_MS;
            boolean pausedLongEnough = pauseTimer.milliseconds() >= MAX_PAUSE_MS;
            if ((atSpeed || (spunUp && pausedLongEnough)) && pauseTimer.milliseconds() >= MIN_PAUSE_MS) {
                feeding = true;
                peakVelocity = velocity;
            }
        }
        transfer.setPower(feeding ? TRANSFER_POWER : 0);
    }

    public void forceFeed() {
        feeding = false;
        transfer.setPower(TRANSFER_POWER);
    }

    public boolean isFeeding() {
        return feeding;
    }

    public void stopFeeder() {
        feeding = false;
        transfer.setPower(0);
    }

    public void stop() {
        goalVelocity = 0;
        stopFeeder();
        outtake.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("goalVelocity", goalVelocity);
        telemetry.addData("curVelocity", "%.0f", velocity);
        telemetry.addData("outtakePower", "%.2f", power);
        telemetry.addData("transfer", feeding ? "FEEDING" : "waiting");
        if (fullPowerTimer.seconds() > 2) {
            telemetry.addLine("Flywheel can't reach goal: lower MAX_VELOCITY");
        }
    }
}
