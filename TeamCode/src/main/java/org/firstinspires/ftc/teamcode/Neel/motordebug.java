package org.firstinspires.ftc.teamcode.Neel;




import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "Motordebug", group = "Glitch")
public class motordebug extends LinearOpMode {

    private CRServo leftServo;
    private IMU imu;
    private DcMotor intake;
    private DcMotor outtake;
    private CRServo transfer;
    private CRServo rightServo;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;


    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();


        while (opModeIsActive()) {
            if (isStopRequested()) return;


            initializeMotors();
            doDriving();
            runmotors();


            telemetry.update();
        }
    }


    private void doDriving() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;



        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    private void runmotors(){
        if (gamepad1.a) {
            imu.resetYaw();
        }
//        if(gamepad1.b){
//            frontRightMotor.setPower(0.2);
//
//        }
        if(gamepad1.x){
            intake.setPower(-0.5);
        }
        if(gamepad1.y){
            intake.setPower(0.5);
        }
        if(gamepad1.dpad_left){
//            outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            outtake.setVelocity(103);
            outtake.setPower(0.5);
        }
        if(gamepad1.dpad_right){
            outtake.setPower(-0.5);
        }
        if(gamepad1.right_bumper){
            outtake.setPower(0);
            transfer.setPower(0);
            intake.setPower(0);
        }
        if(gamepad1.right_trigger>0){
            transfer.setPower(1);
        }
        if(gamepad1.left_trigger>0){
            transfer.setPower(-1);
        }
        if(gamepad1.right_trigger == 0){
            transfer.setPower(0);
        }


       if(gamepad1.dpad_up){
            leftServo.setPower(0.5);
            rightServo.setPower(0.5);
        }
        if(gamepad1.dpad_right){
            leftServo.setPower(-0.25);
            rightServo.setPower(-0.25);
        }
    }

    private void initializeMotors() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        leftServo = hardwareMap.crservo.get("leftServo");
       rightServo = hardwareMap.crservo.get("rightServo");
        intake = hardwareMap.dcMotor.get("intake");
        imu = hardwareMap.get(IMU.class, "imu");
        outtake = hardwareMap.dcMotor.get("outtake");
        transfer = hardwareMap.crservo.get("transfer");



        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftServo.setDirection(CRServo.Direction.REVERSE);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//

    }
}



