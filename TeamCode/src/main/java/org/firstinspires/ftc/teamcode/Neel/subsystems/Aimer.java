package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Aimer {
    public static double KP = 0.015;
    public static double KS = 0.06;
    public static double MAX_TURN = 0.5;
    public static double TOLERANCE_DEG = 0.4;
    public static double UPDATE_MAX_RATE = 5;
    public static double STILL_MS = 120;

    private final ElapsedTime stillTimer = new ElapsedTime();
    private boolean hasAimHeading = false;
    private double aimHeading = 0;
    private double error = 0;

    public void reset() {
        hasAimHeading = false;
    }

    public void updateFromCamera(double heading, double turnRate, double angleToTarget) {
        if (turnRate >= UPDATE_MAX_RATE) {
            stillTimer.reset();
        }
        if (!hasAimHeading || stillTimer.milliseconds() >= STILL_MS) {
            setAimHeading(FieldMap.wrapDegrees(heading - angleToTarget));
        }
    }

    public void noteTurnRate(double turnRate) {
        if (turnRate >= UPDATE_MAX_RATE) {
            stillTimer.reset();
        }
    }

    public void setAimHeading(double heading) {
        aimHeading = heading;
        hasAimHeading = true;
    }

    public boolean hasAim() {
        return hasAimHeading;
    }

    public double turnPower(double heading) {
        if (!hasAimHeading) {
            return 0;
        }
        error = FieldMap.wrapDegrees(heading - aimHeading);
        if (Math.abs(error) <= TOLERANCE_DEG) {
            return 0;
        }
        double rx = KP * error + Math.signum(error) * KS;
        return Math.max(-MAX_TURN, Math.min(MAX_TURN, rx));
    }

    public boolean isAimed() {
        return hasAimHeading && Math.abs(error) <= TOLERANCE_DEG;
    }

    public double getError() {
        return error;
    }
}
