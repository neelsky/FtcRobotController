package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public static double IN_POWER = -0.5;
    public static double OUT_POWER = 0.5;
    public static double SIDES_IN_POWER = -1;
    public static double SIDES_OUT_POWER = 1;
    public static String FLOWER_NAME = "flowerIntake";
    public static double FLOWER_RIGHT_POS = 0;
    public static double FLOWER_LEFT_POS = 0.68;

    private final DcMotorEx intake;
    private Servo flower;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        try {
            flower = hardwareMap.get(Servo.class, FLOWER_NAME);
        } catch (Exception e) {
            flower = null;
        }

        stop();
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }


    public void setFlower(double position) {
        if (flower != null) {
            flower.setPosition(position);
        }
    }

    public void flowerRight() {
        setFlower(FLOWER_RIGHT_POS);
    }

    public void flowerLeft() {
        setFlower(FLOWER_LEFT_POS);
    }

    public boolean hasFlower() {
        return flower != null;
    }

    public void pickup(boolean on) {
        setIntake(on ? IN_POWER : 0);

    }

    public void stop() {
        setIntake(0);
  ;
    }
}
