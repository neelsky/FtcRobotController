package org.firstinspires.ftc.teamcode.raahi.autos.blue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;


@Autonomous(name = "Blue Flower", group = "Auto")
public class blueFlower extends LinearOpMode {


    //PLACEHOLDERS
    double xoffset = 25.4;

    //need to measure offset in mm
    double yoffset = -127;

    private DcMotorEx outtake;

    private CRServo leftIntake;
    private CRServo rightIntake;
    private DcMotorEx mainIntake;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver odometry;

    static final double mmPerFoot   = 304.8;
    static final double mmErrorTolerance  = 25;
    static final double maxPower     = 0.6;
    static final double minPower     = 0.2;
    static final double distanceOffset   = 0.002;
    static final double headingOffset    = 1.5;

    static final double vBase = 941.2069;
    static final double vScale = 0.4127235;
    static final double vExponent = 1.4620166;
    static final double vOffset = 147;



    static final double aimToleranceDeg = 1.5;
    static final double aimKp = 0.02;
    static final double minAimPower = 0.12;
    static final double maxAimPower = 0.4;
    static final double aimOut = 1.5;

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

    double range = 0.02;
    double minRange;
    double maxRange;

    private static final int scoringSideTag = 40;
    private static final int audienceSideTag = 44;
    private static final int aprilTagPipeline = 0;

    private Limelight3A limelight;

    int goalVelocity = 0;

    double distanceToTarget = 0;

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(aprilTagPipeline);



        initializeMotors();
        setUpOdometry();


        waitForStart();
        limelight.start();
        time.reset();




        goalVelocity = velocityForDistance(100);
        outtake.setVelocity(goalVelocity);
        drive(500,500,0,false);
        aim();
        LLResultTypes.FiducialResult upTag = findUpTag();
        if(upTag != null){
            setGoalVelocity(upTag);
            runOuttakeMotor();
        }else{
            goalVelocity = 0;
            outtake.setVelocity(0);

        }
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < 2.0) {
            outtake.setVelocity(goalVelocity);
            if (Math.abs(outtake.getVelocity() - goalVelocity) < goalVelocity * 0.02) break;
        }


        leftIntake.setPower(0.3);
        rightIntake.setPower(0.3);
        mainIntake.setPower(1);
        t.reset();
        while (opModeIsActive() && t.seconds() < 2.0) {
            outtake.setVelocity(goalVelocity);
        }

        setPower(0, 0, 0);
        outtake.setVelocity(0);
        limelight.stop();
        mainIntake.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);









        ;
        double mstime = time.milliseconds();
        double time = mstime / 1000;

        telemetry.addData("Match Time", "%.2f", time);
        telemetry.update();

    }


    private LLResultTypes.FiducialResult findUpTag(){
        LLResultTypes.FiducialResult upTag = null;
        double upTagHeight = Double.NEGATIVE_INFINITY;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (id != scoringSideTag && id != audienceSideTag) {
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
            telemetry.addData("Distance to hive", "%.1f in", distanceToTagInches(upTag));
        }
        telemetry.update();
        return upTag;
    }



    private void setGoalVelocity(LLResultTypes.FiducialResult upTag) {

        distanceToTarget = distanceToTagInches(upTag);
        telemetry.addData("distance", "%.1f in", distanceToTarget);

        if (distanceToTarget > 40 && distanceToTarget < 140) {
            goalVelocity = velocityForDistance(distanceToTarget);
            telemetry.addData("goalVelocity", goalVelocity);
        }
    }

    private int velocityForDistance(double inches) {
        return (int) (vBase + vScale * Math.pow(inches, vExponent) + vOffset);
    }

    private void runOuttakeMotor() {
         double currentVelocity = outtake.getVelocity();
        telemetry.addData("currentVelocity", currentVelocity);

        minRange = goalVelocity - (goalVelocity * range);
        maxRange = goalVelocity;

        isAtGoalVelocity = (currentVelocity <= maxRange) && (currentVelocity > minRange);
        telemetry.addData("isAtGoalVelocity", isAtGoalVelocity);

        if (isAtGoalVelocity) {
            return;
        }

        outtake.setVelocity(goalVelocity);
    }


    private double tagHeightInches(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
        return p.z;
    }


    private double distanceToTagInches(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
        return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    private void initializeMotors() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");



        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();




        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        PIDFCoefficients pidfValues = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Flywheel PIDF",
                "P=%.6f I=%.6f D=%.6f F=%.6f",
                pidfValues.p, pidfValues.i, pidfValues.d, pidfValues.f);
        telemetry.update();

        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        mainIntake = hardwareMap.get(DcMotorEx.class, "mainIntake");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for(DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight, mainIntake}){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    private void setUpOdometry(){
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setOffsets(xoffset, yoffset, DistanceUnit.MM);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.resetPosAndIMU();
        sleep(300);
    }

    private void turn(double degrees){   // >>> ADDED: whole method. drive() always goes to an ABSOLUTE position, so to turn in place we use the current position as the target.
        odometry.update();   // >>> ADDED: refresh the position so we read where the robot is right now
        Pose2D p = odometry.getPosition();   // >>> ADDED: current pose
        drive(p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), Math.toRadians(degrees), false);   // >>> ADDED: target = current x,y so atPosition is true immediately and only the heading part runs. toRadians because drive() works in radians.
    }

    private boolean aim(){
        ElapsedTime aimTimer = new ElapsedTime();

        while(opModeIsActive() && aimTimer.seconds() < aimOut){
            LLResultTypes.FiducialResult tag = findUpTag();
            if (tag == null) {
                setPower(0, 0, 0);
                continue;

            }
            double tagX = tag.getTargetXDegrees();

            if (Math.abs(tagX) < aimToleranceDeg) {   // >>> ADDED: close enough to centered
                setPower(0, 0, 0);   // >>> ADDED: stop turning
                return true;   // >>> ADDED: success
            }

            double turn = tagX * aimKp;
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
            leftIntake.setPower(0.3);
            rightIntake.setPower(0.3);
            mainIntake.setPower(1);
        }else{
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            mainIntake.setPower(0);
        }

        double timeout = Double.MAX_VALUE;
        boolean firstTime = true;
        ElapsedTime driveTimer = new ElapsedTime();


        while(opModeIsActive() &&  driveTimer.seconds() < timeout){
            odometry.update();
            Pose2D currentPosition = odometry.getPosition();
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


            double turn = headingError * headingOffset;
            if (!atHeading && Math.abs(turn) < minTurnPower)turn = Math.signum(headingError) * minTurnPower;
            turn = Math.max(-maxTurnPower, Math.min(maxTurnPower, turn));

            setPower(speedX, speedY, turn);
        }

        setPower(0, 0, 0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        mainIntake.setPower(0);
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



}