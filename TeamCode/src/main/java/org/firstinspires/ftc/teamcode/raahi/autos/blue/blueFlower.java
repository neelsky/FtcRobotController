package org.firstinspires.ftc.teamcode.raahi.autos.blue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;




@Autonomous(name = "Blue Flower", group = "Auto")
public class blueFlower extends LinearOpMode {



    double xoffset = 8;


    double yoffset = -48;

    private DcMotorEx outtake;

    private CRServo leftServo;
    private CRServo rightServo;
    private DcMotorEx intake;

    private CRServo transfer;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;

    static final double goalXMM = 1447.8;
    static final double goalYMM = -406.4;

    static final double outtakeXOffsetIn = 0; // TODO: measure

    static final double vTolerance = 0.03;


    static final double mmPerFoot   = 304.8;
    static final double mmErrorTolerance  = 25;
    static final double maxPower     = 0.6; //TODO
    static final double minPower     = 0.2; //TODO
    static final double distanceOffset   = 0.002; //TODO
    static final double headingOffset    = 1.24; //TODO



    static final double timePerBall = 0.6; //TODO

    static final int maxBalls = 4;

    static final double aimSpinSign = -1; //TODO




    static final double aimToleranceDeg = 1.76; //TODO
    static final double aimKp = 0.02; //TODO
    static final double minAimPower = 0.12; //TODO
    static final double maxAimPower = 0.4; //TODO
    static final double aimOut = 1.5; //TODO

    static final double driveTimeSec = 1.5;
    static final double driveMmPerSec = 600;
    static final double driveOffsetForTime = 2.0;

    static final double maxTurnPower = 0.5;

    static final double minTurnPower = 0.15;



    public static double NEW_P = 7.0;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 15.0;

    boolean isAtGoalVelocity = false;

    boolean outtakeOn = false;

    public static final double pi = Math.PI;





    int goalVelocity = 0;

    double distanceToTarget = 0;

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeMotors();
        setUpOdometry();

        waitForStart();
        time.reset();

        outtakeOn = true;

        drive(0,0,pi/2,false);
        /*drive(goalXMM, goalYMM - 762, 0, false);*/




        aim();


        waitForFlywheel(2.0);

        feedBalls(3);

        outtakeOn = false;

        setPower(0, 0, 0);
        outtake.setVelocity(0);
        intake.setPower(0);
        leftServo.setPower(0);
        rightServo.setPower(0);
        transfer.setPower(0);

        double matchTime = time.seconds();
        telemetry.addData("Match Time", "%.2f", matchTime);
        telemetry.update();
    }


    private double distanceToGoalInches() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        double dx = goalXMM - p.getX(DistanceUnit.MM);
        double dy = goalYMM - p.getY(DistanceUnit.MM);
        return Math.hypot(dx, dy) / 25.4;
    }

    private double headingToGoal() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        double dx = goalXMM - p.getX(DistanceUnit.MM);
        double dy = goalYMM - p.getY(DistanceUnit.MM);
        return Math.atan2(dy, dx);
    }






    private void updateShooter() {
        if (!outtakeOn) return;
        distanceToTarget = distanceToGoalInches() + outtakeXOffsetIn;
        double clamped = Math.max(40, Math.min(140, distanceToTarget));
        goalVelocity = velocityForDistance(clamped);
        outtake.setVelocity(goalVelocity);
    }

    private int velocityForDistance(double inches) {
        goalVelocity = 0;
        if (inches >= 40 && inches <= 140) {
            int fromDistance = (int) Math.round(3.32 * inches + 666);
            goalVelocity = Math.min(fromDistance, 2200);
        } else {
            goalVelocity = 900;
        }
        return (goalVelocity);
    }

    private boolean atGoalVelocity() {
        double v = outtake.getVelocity();
        isAtGoalVelocity = Math.abs(v - goalVelocity) <= goalVelocity * vTolerance;
        return isAtGoalVelocity;
    }

    private void waitForFlywheel(double timeoutSec) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutSec) {
            updateShooter();
            telemetry.addData("dist (in)", "%.1f", distanceToTarget);
            telemetry.addData("goalVelocity", goalVelocity);
            telemetry.addData("currentVelocity", outtake.getVelocity());
            telemetry.update();
            if (atGoalVelocity()) break;
        }
    }




    private void initializeMotors() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");




        telemetry.setMsTransmissionInterval(11);





        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        PIDFCoefficients pidfValues = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Flywheel PIDF",
                "P=%.6f I=%.6f D=%.6f F=%.6f",
                pidfValues.p, pidfValues.i, pidfValues.d, pidfValues.f);
        telemetry.update();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for(DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight, intake}){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    private void setUpOdometry(){
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(xoffset, yoffset, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        sleep(300);

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0,
                AngleUnit.RADIANS, 0));

    }

    private void turn(double degrees){
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        drive(p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), Math.toRadians(degrees), false);
    }

    private boolean aim() {
        ElapsedTime aimTimer = new ElapsedTime();

        while (opModeIsActive() && aimTimer.seconds() < aimOut) {

            updateShooter();
            pinpoint.update();
            double currentHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            double headingError = turnAngle(headingToGoal() - currentHeading);
            double errorDeg = Math.toDegrees(headingError);

            telemetry.addData("aimErrorDeg", "%.2f", errorDeg);
            telemetry.update();

            if (Math.abs(errorDeg) < aimToleranceDeg) {
                setPower(0, 0, 0);
                return true;
            }

            double turn = aimSpinSign * errorDeg * aimKp;
            if (Math.abs(turn) < minAimPower) {
                turn = Math.signum(turn) * minAimPower;
            }
            turn = Math.max(-maxAimPower, Math.min(maxAimPower, turn));
            setPower(0, 0, turn);
        }
        setPower(0, 0, 0);
        return false;
    }

    private void drive(double targetX, double targetY, double targetHeading, boolean intakeHelper){

        if(intakeHelper){
            leftServo.setPower(0.3);
            rightServo.setPower(0.3);
            intake.setPower(1);
        }else{
            leftServo.setPower(0);
            rightServo.setPower(0);
            intake.setPower(0);
        }

        double timeout = Double.MAX_VALUE;
        boolean firstTime = true;
        ElapsedTime driveTimer = new ElapsedTime();


        while(opModeIsActive() &&  driveTimer.seconds() < timeout){
            updateShooter();
            pinpoint.update();
            Pose2D currentPosition = pinpoint.getPosition();
            double currentX = currentPosition.getX(DistanceUnit.MM);
            double currentY = currentPosition.getY(DistanceUnit.MM);
            double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);
            telemetry.addData("currentX", currentX);
            telemetry.addData("currentY", currentY);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.update();



            double xError = targetX - currentX;
            double yError = targetY - currentY;
            double headingError = turnAngle(targetHeading - currentHeading);
            double neededDistance = Math.hypot(xError, yError);

            if (firstTime) {
                timeout = driveTimeSec + (neededDistance / driveMmPerSec) * driveOffsetForTime;
                driveTimer.reset();
                firstTime = false;
            }



            boolean atPosition = neededDistance < mmErrorTolerance;
            boolean atHeading = Math.abs(headingError) < Math.toRadians(2);
            if (atPosition && atHeading) break;


            double speedX = 0;
            double speedY = 0;

            if (!atPosition) {
                double cos = Math.cos(currentHeading);
                double sin = Math.sin(currentHeading);
                double forwardErr =  xError * cos + yError * sin;
                double leftErr = -xError * sin + yError * cos;
                double speed = Math.max(minPower, Math.min(maxPower, neededDistance * distanceOffset));
                speedX = (forwardErr / neededDistance) * speed;
                speedY = (leftErr / neededDistance) * speed;
            }


            double turn = -headingError * headingOffset;
            if (!atHeading && Math.abs(turn) < minTurnPower) turn = Math.signum(turn) * minTurnPower;
            turn = Math.max(-maxTurnPower, Math.min(maxTurnPower, turn));

            setPower(speedX, speedY, turn);
        }

        setPower(0, 0, 0);
        leftServo.setPower(0);
        rightServo.setPower(0);
        intake.setPower(0);
        sleep(100);






    }

    private void setPower(double forward, double left, double turn){
        double frontLeftPowerHelper = forward + left + turn;
        double frontRightPowerHelper = forward - left - turn;
        double backLeftPowerHelper = forward - left + turn;
        double backRightPowerHelper = forward + left - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(frontLeftPowerHelper), Math.abs(frontRightPowerHelper)), Math.max(Math.abs(backLeftPowerHelper), Math.abs(backRightPowerHelper))));

        frontLeft.setPower(frontLeftPowerHelper / max);
        frontRight.setPower(frontRightPowerHelper / max);
        backLeft.setPower(backLeftPowerHelper / max);
        backRight.setPower(backRightPowerHelper / max);
    }

    private double turnAngle(double angleHelper){
        while (angleHelper > Math.PI)  angleHelper -= 2 * Math.PI;
        while (angleHelper < -Math.PI) angleHelper += 2 * Math.PI;
        return angleHelper;
    }

    private void pause(double seconds) {
        ElapsedTime waitTimer = new ElapsedTime();
        while (opModeIsActive() && waitTimer.seconds() < seconds) {
            updateShooter();
            idle();
        }
    }

    private void feedBalls(int count) {
        count = Math.max(1, Math.min(maxBalls, count));
        leftServo.setPower(0.3);
        rightServo.setPower(0.3);
        intake.setPower(0.72);

        for (int i = 0; i < count && opModeIsActive(); i++) {

            transfer.setPower(1);
            pause(timePerBall);

            transfer.setPower(0);
            updateShooter();
            if (i < count - 1) waitForFlywheel(0.5);
        }

        transfer.setPower(0);
        intake.setPower(0);
        leftServo.setPower(0);
        rightServo.setPower(0);
    }








}
