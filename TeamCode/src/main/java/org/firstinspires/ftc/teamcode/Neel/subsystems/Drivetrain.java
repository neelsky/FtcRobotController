package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain {
    private final DcMotor frontLeft;
    private final DcMotor backLeft;
    private final DcMotor frontRight;
    private final DcMotor backRight;
    private IMU imu;

    public Drivetrain(HardwareMap hardwareMap, boolean useImu) {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        backLeft.setDirection(Constants.driveConstants.leftRearMotorDirection);
        frontRight.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        backRight.setDirection(Constants.driveConstants.rightRearMotorDirection);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (useImu) {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP)));
            imu.resetYaw();
        }
    }

    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getTurnRate() {
        return Math.abs(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
    }
}
