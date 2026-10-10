package org.firstinspires.ftc.teamcode.raahi.autos.red;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;


@Autonomous(name = "Red Garden", group = "Auto")
public class redGarden extends LinearOpMode {


    //PLACEHOLDERS
    double xoffset;

    //need to measure offset in mm
    double yoffset;

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

    static final double maxTurnPower = 0.5;

    static final double minTurnPower = 0.15;

    double currentX;
    double currentY;
    double currentHeading;

    public static double NEW_P = 7.0;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 15.0;

    boolean isAtGoalVelocity = false;

    double range = 0.02;
    double minRange;
    double maxRange;

    private static final int scoringSideTag = 32;
    private static final int audienceSideTag = 36;
    private static final int aprilTagPipeline = 0;

    private Limelight3A limelight;

    int goalVelocity = 0;

    double distanceToTarget = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(aprilTagPipeline);

        initializeMotors();
        setUpOdometry();


        waitForStart();
        limelight.start();



        /*while (opModeIsActive()) {
            LLResultTypes.FiducialResult upTag = findUpTag();
            if(upTag != null){
                setGoalVelocity(upTag);
                runOuttakeMotor();
            }else{
                goalVelocity = 0;
                outtake.setVelocity(0);

            }
        }*/

        drive(2000,0,0, false);
        sleep(1000);
        drive(0,2000,0, true);

        sleep(1000);
        drive(2000,2000,0, true);

        sleep(1000);
        drive(0,0,0, false);

        turn(30);

        limelight.stop();
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
            goalVelocity = (int) (941.2069 + 0.4127235 * Math.pow(distanceToTarget, 1.4620166) + 147);
            telemetry.addData("goalVelocity", goalVelocity);
        }
    }

    private void runOuttakeMotor() {
        double currentVelocity = outtake.getVelocity();
        telemetry.addData("curVelocity", currentVelocity);

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


        while(opModeIsActive()){
            odometry.update();
            Pose2D currentPosition = odometry.getPosition();
            double currentX = currentPosition.getX(DistanceUnit.MM);
            double currentY = currentPosition.getY(DistanceUnit.MM);
            double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);


            double xError = targetX - currentX;
            double yError = targetY - currentY;
            double headingError = turnAngle(targetHeading - currentHeading);
            double neededDistance = Math.hypot(xError, yError);



            boolean atPosition = neededDistance < mmErrorTolerance;
            boolean atHeading = Math.abs(headingError) < Math.toRadians(2);
            if (atPosition && atHeading) break;


            double speedX = 0;
            double speedY = 0;

            if (!atPosition) {
                double forwardErr =  xError * Math.cos(currentHeading) + yError * Math.sin(currentHeading);
                double leftErr = -xError * Math.sin(currentHeading) + yError * Math.cos(currentHeading);
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