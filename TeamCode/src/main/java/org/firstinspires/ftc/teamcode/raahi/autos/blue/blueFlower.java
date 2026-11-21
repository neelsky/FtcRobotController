package org.firstinspires.ftc.teamcode.raahi.autos.blue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;


import org.firstinspires.ftc.teamcode.Neel.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Blue Flower 6", group = "Auto")
public class blueFlower extends LinearOpMode {



    double xoffset = 8;


    double yoffset = -48;

    private DcMotorEx outtake;




    private DcMotorEx intake;

    private DcMotor transfer;

    private Servo turret;

    private Limelight3A limelight;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private IMU imu;
    private GoBildaPinpointDriver pinpoint;

    private final Aimer aimer = new Aimer();
    private Shooter shooter;




    static final double goalXMM = 1270;
    static final double goalYMM = 0;
    static final double goalOneXMM = 1270;
    static final double goalOneYMM = 0;
    static final double goalTwoXMM = 2170;
    static final double goalTwoYMM = 0;

    static final int firstTag = 40;
    static final int secondTag = 44;

    int turretTagOne = 40;
    int turretTagTwo = 44;

    static final double outtakeXOffsetIn = 0;

    static final double vTolerance = 0.03;

    static final double mmPerFoot = 304.8;
    static final double mmErrorTolerance = 13.2;
    static final double maxPower = 0.6;
    static final double minPower = 0.12;
    static final double distanceOffset = 0.002;
    static final double headingOffset = 1.24;

    static final double timePerBall = 1.08;
    static final int maxBalls = 4;

    static final double tileLength = 584.2;

    static final double startFieldRelativeXMM = 203.2;
    static final double startFieldRelativeYMM = -1422.4;
    boolean visionSynced = false;

    static final double aimToleranceDeg = 1.26;
    static final double aimKp = 0.02;
    static final double minAimPower = 0.04;
    static final double maxAimPower = 0.4;
    static final double aimOut = 1.5;

    static final double driveTimeSec = 1.5;
    static final double driveMmPerSec = 600;
    static final double driveOffsetForTime = 2.0;

    static final double maxTurnPower = 0.5;

    static final double minTurnPower = 0.04;



    public static double NEW_P = 0.0042;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 0.00072;


    public static final double turretCenterPos = 0.5;
    public static final double turretTotalServoDegrees = 360.0;
    public static final double turretMaxSweepDeg = 180.0;

    public static final double turretMinPos = Range.clip(turretCenterPos - (turretMaxSweepDeg / turretTotalServoDegrees), 0.0, 1.0);
    public static final double turretMaxPos = Range.clip(turretCenterPos + (turretMaxSweepDeg / turretTotalServoDegrees), 0.0, 1.0);

    boolean isAtGoalVelocity = false;

    boolean outtakeOn = false;

    public static final double pi = Math.PI;

    public static final double parkXMM = 2336.8;
    public static final double parkYMM = 1305.56;



    int goalVelocity = 0;

    double distanceToTarget = 0;

    private final ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {

        initializeMotors();
        setUpOdometry();

        // pinpoint setup
        pinpoint.update();
        Pose2D currentPosition = pinpoint.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);

        // subsystems

        telemetry.addData("currentX", currentX);
        telemetry.addData("currentY", currentY);
        telemetry.update();

        waitForStart();
        time.reset();

        outtakeOn = true;


        //aim();
        // turretAim(firstTag, goalOneXMM, goalOneYMM);

        // initial shooting
        /*waitForFlywheel(2.0);
        feedBalls(4);*/

        runShooter(true,firstTag);

        //

        everythingOff();


        drive(-78.74,922,pi/2, false, false);
        drive(-88.9,1338.58, pi/2, true, false);
        drive(0,0,-pi/4, false, true);
        LLResultTypes.FiducialResult upTag = findUpTag();
        if(time.seconds() > 22){
            drive(tileLength, tileLength, 0, true, false);
            park();
            return;
        }
        if (upTag != null && upTag.getFiducialId() == firstTag) {
            drive(0,0,0,false, true);
            outtakeOn = true;
            //aim();
            // Alternative turretAim(firstTag, goalOneXMM, goalOneYMM);
            //waitForFlywheel(1.2);
            if(time.seconds() > 25){
                drive(tileLength, tileLength, 0, true, false);
                park();

            }else {
                //feedBalls(4);
                runShooter(true, firstTag);
                everythingOff();
                drive(tileLength, tileLength, 0, false, false);
                park();

            }

        } else if (upTag == null || upTag.getFiducialId() == secondTag) {
            outtakeOn = true;
            drive(1320.8,962.66, 0, false, true);
            drive(2778.76, 462.28, 0, false, true);
            drive(3009.9, 25.4, pi, false, true);
            //aim();
            // Alternative turretAim(secondTag, goalTwoXMM, goalTwoYMM);
            //waitForFlywheel(0.5);
            if(time.seconds() > 27){
                park();
            }else {
                //feedBalls(4);
                runShooter(true,secondTag);
                everythingOff();
                park();
            }


        }


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
        double clamped = Math.max(32, Math.min(114, distanceToTarget));
        goalVelocity = velocityForDistance(clamped);
        outtake.setVelocity(goalVelocity);
    }

    private int velocityForDistance(double inches) {
        goalVelocity = 0;
        if (inches >= 32 && inches <= 114) {
            int fromDistance = (int) Math.round(7.086 * inches + 666);
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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

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
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        turret = hardwareMap.get(Servo.class, "turret");
        turret.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        turret.setPosition(turretCenterPos);

        shooter = new Shooter(hardwareMap);



        //intialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();


        frontLeft.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        backLeft.setDirection(Constants.driveConstants.leftRearMotorDirection);
        frontRight.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        backRight.setDirection(Constants.driveConstants.rightRearMotorDirection);

        for(DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight, intake}){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    private void setUpOdometry(){
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(xoffset, yoffset, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        sleep(300);

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0,
                AngleUnit.RADIANS, 0));

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

            double turn = errorDeg * aimKp;
            if (Math.abs(turn) < minAimPower) {
                turn = Math.signum(turn) * minAimPower;
            }
            turn = Math.max(-maxAimPower, Math.min(maxAimPower, turn));
            setPower(0, 0, turn);
        }
        setPower(0, 0, 0);
        return false;
    }

    private void drive(double targetX, double targetY, double targetHeading, boolean intakeHelper, boolean outtakeHelper){

        if(intakeHelper){

            intake.setPower(0.5);
            transfer.setPower(-0.3);
        }else{
            transfer.setPower(0);
            intake.setPower(0);
        }
        if(outtakeHelper){
            outtake.setVelocity(900);
            outtakeOn = true;
        }else{
            outtake.setVelocity(0);
        }


        //subsystem aiming
        /*double turnRate = getTurnRate();
        if(!outtakeHelper){
            aimer.reset();
            aimer.noteTurnRate(turnRate);
        } else if (findUpTag() != null) {
            aimer.updateFromCamera(heading, turnRate, vision.getAngle());
        } else {
            aimer.noteTurnRate(turnRate);
        }*/


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
            if (atPosition && atHeading){
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                intake.setPower(0);

                sleep(100);
                return;
            }


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


            double turn = headingError * headingOffset;
            if (!atHeading && Math.abs(turn) < minTurnPower) turn = Math.signum(turn) * minTurnPower;
            turn = Math.max(-maxTurnPower, Math.min(maxTurnPower, turn));

            setPower(speedX, speedY, turn);


        }

        setPower(0, 0, 0);

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

        intake.setPower(-0.5);

        for (int i = 0; i < count && opModeIsActive(); i++) {

            transfer.setPower(1);
            pause(timePerBall);

            transfer.setPower(0);
            updateShooter();
            if (i < count - 1) waitForFlywheel(0.5);
        }

        transfer.setPower(0);

        intake.setPower(0);
    }

    private void everythingOff(){

        outtakeOn = false;
        setPower(0, 0, 0);
        outtake.setVelocity(0);
        intake.setPower(0);

        transfer.setPower(0);
    }

    private LLResultTypes.FiducialResult findUpTag(){
        LLResultTypes.FiducialResult upTag = null;
        double upTagHeight = Double.NEGATIVE_INFINITY;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (!visionSynced) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double absoluteForwardMM = botpose.getPosition().y * 1000.0;
                    double absoluteSidewaysMM = botpose.getPosition().x * 1000.0;
                    double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                    double relativeXMM = absoluteForwardMM - startFieldRelativeXMM;
                    double relativeYMM = absoluteSidewaysMM - startFieldRelativeYMM;

                    pinpoint.setPosition(new Pose2D(DistanceUnit.MM, relativeXMM, relativeYMM, AngleUnit.RADIANS, headingRad));
                    visionSynced = true;
                }
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (id != firstTag && id != secondTag) {
                    continue;
                }

                double height = tagHeightInches(tag);
                if (upTag == null || height > upTagHeight) {
                    upTag = tag;
                    upTagHeight = height;

                }
            }
        }

        if (upTag == null) {
            telemetry.addData("Distance to hive", "no tag in view");
        } else {
            telemetry.addData("Distance to hive", "%.1f in", upTagHeight);
        }
        telemetry.update();
        return upTag;
    }

    private double tagHeightInches(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
        return p.z;
    }

    private void park(){
        drive(parkXMM, parkYMM, 0, true, false);
    }

    private boolean turretAim(int tag, double tagXMM, double tagYMM) {
        boolean tagFound = false;
        double tx = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> currentDetections = result.getFiducialResults();
            for (LLResultTypes.FiducialResult detection : currentDetections) {
                if (detection.getFiducialId() == tag) {
                    tagFound = true;
                    tx = detection.getTargetXDegrees();
                    break;
                }
            }
        }
        if (tagFound) {
            ElapsedTime turretTimer = new ElapsedTime();
            double timeoutSec = 2.5;
            double kP = 0.01;

            while (opModeIsActive() && turretTimer.seconds() < timeoutSec && Math.abs(tx) > 2.0) {
                result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> currentDetections = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult detection : currentDetections) {
                        if (detection.getFiducialId() == tag) {
                            tx = detection.getTargetXDegrees();
                            break;
                        }
                    }

                    double targetPos = Range.clip(turret.getPosition() + ((tx * kP) / turretTotalServoDegrees), turretMinPos, turretMaxPos);
                    turret.setPosition(targetPos);
                } else {
                    break;
                }
            }
        } else {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double robotXMM = pose.getX(DistanceUnit.MM);
            double robotYMM = pose.getY(DistanceUnit.MM);
            double robotHeading = pose.getHeading(AngleUnit.RADIANS);

            double dx = tagXMM - robotXMM;
            double dy = tagYMM - robotYMM;
            double fieldAngleToTag = Math.atan2(dy, dx);

            double relativeAngleRad = turnAngle(fieldAngleToTag - robotHeading);
            double relativeAngleDeg = Math.toDegrees(relativeAngleRad);


            double targetPos = Range.clip(turretCenterPos + (relativeAngleDeg / turretTotalServoDegrees), turretMinPos, turretMaxPos);
            turret.setPosition(targetPos);
        }

        return tagFound;
    }

    public double getTurnRate() {

        return Math.abs(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);

    }


    private void runShooter(boolean shooting, int tagID) {
        if (!shooting) {
            shooter.setGoal(0);
        } else if (findUpTag() != null) {
            shooter.setGoalFromDistance(distanceToTag(tagID));
        } else {
            shooter.useDefaultIfOff();
        }
        shooter.update();

        if (gamepad1.left_trigger > 0.1) {
            shooter.forceFeed();
        } else {
            shooter.runFeeder(shooting);
        }
    }



    private double distanceToTag(int tagID){
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : fiducials) {
            if (tag.getFiducialId() == 40) {

                Pose3D tagPose = tag.getTargetPoseRobotSpace();

                double x = tagPose.getPosition().x;
                double y = tagPose.getPosition().y;
                double z = tagPose.getPosition().z;

                double distanceMeters = Math.sqrt(x * x + y * y + z * z);
                distanceMeters = distanceMeters * 39.3701;
                return distanceMeters;
            }
        }
        return 0;
    }




}
