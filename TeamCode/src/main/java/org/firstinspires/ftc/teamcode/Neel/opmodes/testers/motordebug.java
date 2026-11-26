package org.firstinspires.ftc.teamcode.Neel.opmodes.testers;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Motordebug", group = "Glitch")
public class motordebug extends LinearOpMode {

    private Servo flowerIntake;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx transfer;
    private DcMotorEx outtake;
    private DcMotor intake;


    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();


        while (opModeIsActive()) {
            if (isStopRequested()) return;


            initializeMotors();
            doDriving();
            runMotors();



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
    private void runMotors(){
        if(gamepad1.a) {
            transfer.setVelocity(-1200);
        }
        if(gamepad1.b){
            transfer.setVelocity(560);
        }
        if(gamepad1.dpad_left){
            outtake.setVelocity(800);
        }
        if (gamepad1.dpad_right) {

            outtake.setVelocity(800);
        }
        if (gamepad1.right_bumper){
            intake.setPower(1);
        }
        if(gamepad1.left_bumper){
            intake.setPower(-1);
        }
        if(gamepad1.dpad_up){
            flowerIntake.setPosition(0.68);
        }
        if(gamepad1.dpad_down){
            flowerIntake.setPosition(0);
        }
        if(gamepad1.back){
            flowerIntake.setPosition(0);
            transfer.setPower(0);
        }
    }

    private void initializeMotors() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake = hardwareMap.dcMotor.get("intake");
        flowerIntake = hardwareMap.servo.get("flowerIntake");




        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);



        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//

    }
}



