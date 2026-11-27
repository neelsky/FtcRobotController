package org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    public static double P = 0.005;
    public static double I = 0.0003;
    public static double D = 0;
    public static double F = 0.0012;
    public static double MAX_I_POWER = 0.2;
    public static double RESET_I_ON_GOAL_CHANGE = 100;

    public static double TABLE_START_IN = 25;
    public static double TABLE_STEP_IN = 5;
    public static int[] SPEED_TABLE = {
            693, 701, 708, 715, 723, 730, 702,
            735, 748, 760, 773, 786, 798, 781,
            764, 866, 873, 881, 888, 895, 903
    };
    public static int DEFAULT_VELOCITY = 600;
    public static int MAX_VELOCITY = 1000;
    public static double READY_AT = 0.97;
    public static double FULL_POWER_BELOW = 0.95;
    public static double NOMINAL_VOLTS = 12.0;
    public static double MAXED_OUT_S = 0.5;
    public static double MAXED_OUT_RISE = 0.01;

    private final DcMotorEx outtake;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private int goalVelocity = 0;
    private int lastGoal = 0;
    private double velocity = 0;
    private double power = 0;
    private double integral = 0;
    private double lastError = 0;
    private final VoltageSensor battery;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double volts = NOMINAL_VOLTS;
    private final ElapsedTime fullPowerTimer = new ElapsedTime();
    private double velocityAtFullPowerStart = 0;
    private boolean fullPowerBefore = false;
    private boolean maxedOut = false;

    public Shooter(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setPower(0);
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
        readVolts();
    }

    private void readVolts() {
        if (battery == null) {
            return;
        }
        double reading = battery.getVoltage();
        if (reading > 6 && reading < 16) {
            volts = reading;
        }
        voltageTimer.reset();
    }

    public static int velocityForDistance(double distanceIn) {
        if (Double.isNaN(distanceIn) || distanceIn <= 0) {
            return -1;
        }
        double spot = (distanceIn - TABLE_START_IN) / TABLE_STEP_IN;
        spot = Math.max(0, Math.min(SPEED_TABLE.length - 1, spot));
        int below = (int) Math.floor(spot);
        int above = Math.min(below + 1, SPEED_TABLE.length - 1);
        double between = spot - below;
        double speed = SPEED_TABLE[below] + between * (SPEED_TABLE[above] - SPEED_TABLE[below]);
        return Math.min((int) Math.round(speed), MAX_VELOCITY);
    }

    public static int tablePointFor(double distanceIn) {
        int index = (int) Math.round((distanceIn - TABLE_START_IN) / TABLE_STEP_IN);
        return Math.max(0, Math.min(SPEED_TABLE.length - 1, index));
    }

    public static double tableDistance(int index) {
        return TABLE_START_IN + index * TABLE_STEP_IN;
    }

    public void setGoal(int velocity) {
        goalVelocity = Math.max(0, Math.min(velocity, MAX_VELOCITY));
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

    public double getPower() {
        return power;
    }

    public boolean atSpeed() {
        return goalVelocity > 0 && velocity >= goalVelocity * READY_AT && velocity <= goalVelocity * (2 - READY_AT);
    }

    public boolean isMaxedOut() {
        return maxedOut;
    }

    public double getVolts() {
        return volts;
    }

    public void update() {
        velocity = Math.abs(outtake.getVelocity());
        double dt = Math.max(1e-3, Math.min(0.1, loopTimer.seconds()));
        loopTimer.reset();
        if (voltageTimer.seconds() > 0.25) {
            readVolts();
        }
        if (goalVelocity <= 0 || Math.abs(goalVelocity - lastGoal) > RESET_I_ON_GOAL_CHANGE) {
            integral = 0;
            lastError = goalVelocity - velocity;
            maxedOut = false;
        }
        lastGoal = goalVelocity;
        power = 0;
        if (goalVelocity > 0) {
            double error = goalVelocity - velocity;
            double derivative = (error - lastError) / dt;
            lastError = error;
            if (velocity < goalVelocity * FULL_POWER_BELOW) {
                power = 1;
            } else {
                integral += error * dt;
                if (I > 0) {
                    double maxIntegral = MAX_I_POWER / I;
                    integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
                }
                double scale = NOMINAL_VOLTS / volts;
                power = F * goalVelocity * scale + P * error + I * integral + D * derivative;
                power = Math.max(0, Math.min(1, power));
            }
        }
        watchForMaxedOut();
        outtake.setPower(power);
    }

    private void watchForMaxedOut() {
        boolean fullPower = power >= 0.99 && velocity < goalVelocity * READY_AT;
        if (!fullPower) {
            fullPowerBefore = false;
            maxedOut = false;
            return;
        }
        if (!fullPowerBefore) {
            fullPowerBefore = true;
            fullPowerTimer.reset();
            velocityAtFullPowerStart = velocity;
        } else if (fullPowerTimer.seconds() >= MAXED_OUT_S) {
            maxedOut = velocity < velocityAtFullPowerStart * (1 + MAXED_OUT_RISE);
            fullPowerTimer.reset();
            velocityAtFullPowerStart = velocity;
        }
    }

    public void stop() {
        goalVelocity = 0;
        power = 0;
        outtake.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("flywheel", "goal %d | now %.0f | power %.2f | battery %.1f V", goalVelocity, velocity, power, volts);
        if (maxedOut) {
            telemetry.addLine(String.format("FLYWHEEL MAXED OUT at %.0f, can't reach %d: charge the battery or lower that speed", velocity, goalVelocity));
        }
    }
}
