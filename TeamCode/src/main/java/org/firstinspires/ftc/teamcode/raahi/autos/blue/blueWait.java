package org.firstinspires.ftc.teamcode.raahi.autos.blue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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


import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Blue Wait V5.3.4", group = "Auto")
public class blueWait extends LinearOpMode {



    // Pinpoint takes the forward pod's Y offset first, then the strafe pod's X offset.
    double xoffset = -15.0;
    double yoffset = -106.0;

    private DcMotorEx outtake;




    private DcMotorEx intake;

    private DcMotor transfer;

    private Servo turret;

    private Limelight3A limelight;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private GoBildaPinpointDriver pinpoint;

    private Shooter shooter;




    static final double goalXMM = 1270;
    static final double goalYMM = 0;
    static final double goalOneXMM = 1270;
    static final double goalOneYMM = 0;
    static final double goalTwoXMM = 2170;
    static final double goalTwoYMM = 0;

    static final int firstTag = 40;
    static final int secondTag = 44;


    static final double outtakeXOffsetIn = 0;



    static final double mmPerFoot = 304.8;
    static final double mmErrorTolerance = 13.2;
    static final double maxPower = 0.6;
    static final double minPower = 0.12;
    static final double distanceOffset = 0.002;
    static final double headingOffset = 1.24;

    static final double timePerBall = 1.08;
    static final int maxBalls = 4;

    static final double tileLength = 584.2;

    private long lastTurretFrameMs = -1;

    // Issue 4: These fields supported the camera pose reset. Pinpoint now keeps its own pose.
    // static final double startFieldRelativeXMM = 203.2;
    // static final double startFieldRelativeYMM = -1422.4;
    // boolean visionSynced = false;

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



    boolean outtakeOn = false;

    public static final double pi = Math.PI;

    public static final double parkXMM = 2336.8;
    public static final double parkYMM = 1305.56;





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


        LLResultTypes.FiducialResult upTag = findUpTag();
        while(opModeIsActive() && (upTag == null || upTag.getFiducialId() != secondTag)){
            sleep(50);
            upTag = findUpTag();
            if (time.seconds() > 23){
                drive(300, 0, pi, false, false, secondTag, goalOneXMM, goalOneYMM);
                drive(300,-1100, pi,false,false,secondTag, goalOneXMM, goalOneYMM);
                return;
            }
        }
        outtakeOn = true;
        runShooter(true, secondTag, false);
        drive(300, 0, pi, false, false, secondTag, goalOneXMM, goalOneYMM);
        drive(300,-1100, pi,false,false,secondTag, goalOneXMM, goalOneYMM);




        //aim();
        // turretAim(firstTag, goalOneXMM, goalOneYMM);

        // initial shooting
        /*waitForFlywheel(2.0);
        feedBalls(4);*/

        // The centered turret already points at the goal in the starting position.






    }




    private double headingToGoal() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        double dx = goalXMM - p.getX(DistanceUnit.MM);
        double dy = goalYMM - p.getY(DistanceUnit.MM);
        return Math.atan2(dy, dx);
    }


    private void updateShooter(int tagID) {
        if (!outtakeOn) {
            shooter.setGoal(0);
        } else {
            distanceToTarget = distanceToTag(tagID) + outtakeXOffsetIn;
            int baseVelocity = Shooter.velocityForDistance(distanceToTarget);
            if (baseVelocity > 0) {
                shooter.setGoal(baseVelocity + 70);
            } else if (shooter.getGoal() == 0) {
                shooter.setGoal(Shooter.DEFAULT_VELOCITY + 50);
            }
        }
        shooter.update();
    }

    private void waitForFlywheel(double timeoutSec, int tagID) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutSec) {
            updateShooter(tagID);
            telemetry.addData("dist (in)", "%.1f", distanceToTarget);
            telemetry.addData("goalVelocity", shooter.getGoal());
            telemetry.addData("currentVelocity", shooter.getVelocity());
            telemetry.update();
            if (Math.abs(shooter.getVelocity() - shooter.getGoal()) <= 50) break;
        }
    }




    private void initializeMotors() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");




        telemetry.setMsTransmissionInterval(11);





        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

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
        // Keep the motor direction used by TeleOp. Shooter shares this transfer motor.
        // transfer.setDirection(DcMotor.Direction.REVERSE);

        turret.setPosition(turretCenterPos);

        shooter = new Shooter(hardwareMap);



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

        // The chassis starts facing backward relative to the waypoint coordinate frame.
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0,
                AngleUnit.RADIANS, Math.PI));

    }



    /*private boolean aim() {
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
    }*/

    private void drive(double targetX, double targetY, double targetHeading, boolean intakeHelper, boolean outtakeHelper, int tag, double tagXMM, double tagYMM){

        if(intakeHelper){

            intake.setPower(0.5);
            // Positive power moves the transfer in the pickup direction used by TeleOp.
            // transfer.setPower(-0.3);
            transfer.setPower(0.3);
        }else{
            transfer.setPower(0);
            intake.setPower(0);
        }
        if(outtakeHelper){
            outtakeOn = true;
        }else{
            outtakeOn = false;
        }




        double timeout = Double.MAX_VALUE;
        boolean firstTime = true;
        ElapsedTime driveTimer = new ElapsedTime();
        ElapsedTime driveTelemetryTimer = new ElapsedTime();


        while(opModeIsActive() &&  driveTimer.seconds() < timeout){
            turretAim(tag, tagXMM, tagYMM);
            updateShooter(tag);
            pinpoint.update();
            Pose2D currentPosition = pinpoint.getPosition();
            double currentX = currentPosition.getX(DistanceUnit.MM);
            double currentY = currentPosition.getY(DistanceUnit.MM);
            double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);



            double xError = targetX - currentX;
            double yError = targetY - currentY;
            double headingError = turnAngle(targetHeading - currentHeading);
            double neededDistance = Math.hypot(xError, yError);

            if (firstTime || driveTelemetryTimer.milliseconds() >= 100) {
                telemetry.addData("Pinpoint pose", "X %.0f mm, Y %.0f mm, heading %.1f deg",
                        currentX, currentY, Math.toDegrees(currentHeading));
                telemetry.addData("Drive target", "X %.0f mm, Y %.0f mm, heading %.1f deg",
                        targetX, targetY, Math.toDegrees(targetHeading));
                telemetry.addData("Drive error", "X %+.0f mm, Y %+.0f mm, heading %+.1f deg",
                        xError, yError, Math.toDegrees(headingError));
                telemetry.update();
                driveTelemetryTimer.reset();
            }

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

            // Issue 2: Convert Pinpoint's left and counterclockwise signs to the wheel mixer's signs.
            // setPower(speedX, speedY, turn);
            setPower(speedX, -speedY, -turn);


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

    /*private void pause(double seconds,) {
        ElapsedTime waitTimer = new ElapsedTime();
        while (opModeIsActive() && waitTimer.seconds() < seconds) {
            updateShooter();
            idle();
        }
    }*/

    /*private void feedBalls(int count) {
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
    }*/

    private void everythingOff(){

        outtakeOn = false;
        setPower(0, 0, 0);
        shooter.stop();
        intake.setPower(0);
    }

    private LLResultTypes.FiducialResult findUpTag(){
        LLResultTypes.FiducialResult upTag = null;
        double upTagHeight = Double.NEGATIVE_INFINITY;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Issue 4: Leave the pose with Pinpoint. A missing camera pose can appear as zeros.
            /*
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
            */

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null) {
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
        drive(parkXMM, parkYMM, 0, true, false, secondTag, goalTwoXMM, goalTwoYMM);
    }

    private boolean turretAim(int tag, double tagXMM, double tagYMM) {
        boolean tagFound = false;
        double tx = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> currentDetections = result.getFiducialResults();
            if (currentDetections != null) {
                for (LLResultTypes.FiducialResult detection : currentDetections) {
                    if (detection.getFiducialId() == tag) {
                        tagFound = true;
                        tx = detection.getTargetXDegrees();
                        break;
                    }
                }
            }
        }
        if (tagFound) {
            // Issue 3: The old loop could block drive and feeder updates for up to 2.5 seconds.
            /*
            ElapsedTime turretTimer = new ElapsedTime();
            double timeoutSec = 2.5;
            double kP = 0.01;

            while (opModeIsActive() && turretTimer.seconds() < timeoutSec && Math.abs(tx) > 2.0) {
                result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> currentDetections = result.getFiducialResults();
                    if (currentDetections != null) {
                        for (LLResultTypes.FiducialResult detection : currentDetections) {
                            if (detection.getFiducialId() == tag) {
                                tx = detection.getTargetXDegrees();
                                break;
                            }
                        }
                    }

                    double targetPos = Range.clip(turret.getPosition() + ((tx * kP) / turretTotalServoDegrees), turretMinPos, turretMaxPos);
                    turret.setPosition(targetPos);
                } else {
                    break;
                }
            }
            */

            // Use each camera frame once so the drive and feeder loops can keep running.
            long frameMs = result.getControlHubTimeStamp();
            if (frameMs != lastTurretFrameMs) {
                lastTurretFrameMs = frameMs;
                if (Math.abs(tx) > 2.0) {
                    double adjustment = 0.10 * tx / turretTotalServoDegrees;
                    turret.setPosition(Range.clip(
                            turret.getPosition() + adjustment, turretMinPos, turretMaxPos));
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

    private void runShooter(boolean shooting, int tagID, boolean aimDuringShot) {
        if (!shooting) {
            shooter.stop();
            return;
        }
        outtakeOn = true;

        ElapsedTime flywheelTimer = new ElapsedTime();
        double timeoutSec = 2.0;

        while (opModeIsActive() && flywheelTimer.seconds() < timeoutSec) {
            if (aimDuringShot) {
                turretAim(tagID, (tagID == firstTag ? goalOneXMM : goalTwoXMM), (tagID == firstTag ? goalOneYMM : goalTwoYMM));
            }
            updateShooter(tagID);
            if (Math.abs(shooter.getVelocity() - shooter.getGoal()) <= 50) {
                break;
            }
            sleep(50);
        }

        shooter.startFeeding();
        ElapsedTime shootTimer = new ElapsedTime();
        double shootSeconds = 5;

        try {
            if (opModeIsActive()) {
                // Match the intake direction used by the original shooting routine.
                intake.setPower(-0.5);
            }

            while (opModeIsActive() && shootTimer.seconds() < shootSeconds) {
                if (aimDuringShot) {
                    turretAim(tagID, (tagID == firstTag ? goalOneXMM : goalTwoXMM), (tagID == firstTag ? goalOneYMM : goalTwoYMM));
                }
                updateShooter(tagID);
                shooter.runFeeder(true);
                shooter.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            intake.setPower(0);
            shooter.stopFeeder();
        }
    }



    private double distanceToTag(int tagID){
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return 0;
        }
        for (LLResultTypes.FiducialResult tag : fiducials) {
            if (tag.getFiducialId() == tagID) {

                Pose3D tagPose = tag.getTargetPoseRobotSpace();
                if (tagPose != null) {
                    double x = tagPose.getPosition().x;
                    double y = tagPose.getPosition().y;
                    double z = tagPose.getPosition().z;

                    double distanceMeters = Math.sqrt(x * x + y * y + z * z);
                    distanceMeters = distanceMeters * 39.3701;
                    return distanceMeters;
                }
            }
        }
        return 0;
    }




}
