package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static double IN_POWER = -0.5;
    public static double OUT_POWER = 0.5;
    public static double SIDES_IN_POWER = -1;
    public static double SIDES_OUT_POWER = 1;

    private final DcMotorEx intake;
    private final CRServo leftServo;
    private final CRServo rightServo;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftServo = hardwareMap.crservo.get("leftServo");
        rightServo = hardwareMap.crservo.get("rightServo");
        leftServo.setDirection(CRServo.Direction.REVERSE);
        stop();
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setSides(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    public void pickup(boolean on) {
        setIntake(on ? IN_POWER : 0);
        setSides(on ? SIDES_IN_POWER : 0);
    }

    public void stop() {
        setIntake(0);
        setSides(0);
    }
}
