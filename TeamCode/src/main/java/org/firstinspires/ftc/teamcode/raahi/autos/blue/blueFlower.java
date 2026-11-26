package org.firstinspires.ftc.teamcode.raahi.autos.blue;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;


import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Neel.subsystems.TurretAimAuto;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.Vision;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Blue Flower V1.8.3", group = "Auto")
public class blueFlower extends LinearOpMode {



    // Pinpoint takes the forward pod's Y offset first, then the strafe pod's X offset.
    double xoffset = -15.0;
    double yoffset = -106.0;

    private DcMotorEx outtake;




    private DcMotorEx intake;

    private DcMotor transfer;

    private TurretAimAuto aim;
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

    static final int firstTag = 44;
    static final int secondTag = 40;


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


    boolean outtakeOn = false;

    public static final double pi = Math.PI;

    public static final double parkXMM = 2336.8;
    public static final double parkYMM = 695.56;





    double distanceToTarget = 0;

    private final ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {

        initializeMotors();
        setUpOdometry();

        pinpoint.update();
        aim = new TurretAimAuto(hardwareMap, Alliance.BLUE, fieldPose(pinpoint.getPosition()));
        boolean manualCamera = aim.getVision() == null;
        if (manualCamera) {
            limelight.pipelineSwitch(Vision.PIPELINE);
            limelight.start();
        }

        try {
            telemetry.addLine("Blue Flower ready");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) {
                return;
            }
            time.reset();
            turret.setPosition(TurretAimAuto.TURRET_ZERO_POS);
            if (turret instanceof PwmControl) {
                ((PwmControl) turret).setPwmEnable();
            }
            aim.start();
            runRoute();
        } finally {
            everythingOff();
            aim.stop();
            if (manualCamera) {
                limelight.stop();
            }
        }
    }

    private void runRoute() {
        runShooter(firstTag);

        everythingOff();

        drive(0, -610, 3 * pi / 2, true, false, firstTag);
        drive(0, 0, pi / 2, true, false, firstTag);


        LLResultTypes.FiducialResult upTag = findUpTag();
        if(time.seconds() > 22){
            park();
            return;
        }
        if (upTag != null && upTag.getFiducialId() == firstTag) {

            outtakeOn = true;
            if(time.seconds() > 25){
                drive(tileLength, tileLength, 0, true, false, secondTag);
                park();

            }else {
                runShooter(firstTag);
                everythingOff();
                park();

            }

        } else if (upTag == null || upTag.getFiducialId() == secondTag) {
            outtakeOn = true;
            drive(1320.8,352.66, 0, false, true, secondTag);
            drive(2778.76, 162.28, 0, false, true, secondTag);
            drive(3009.9, 25.4, pi, false, true, secondTag);
            if(time.seconds() > 27){
                park();
            }else {
                runShooter(secondTag);
                everythingOff();
                park();
            }


        }


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
            double tagDistance = distanceToTag(tagID);
            if (tagDistance <= 0 && aim.isAimKnown()) {
                tagDistance = aim.getDistanceToCell();
            }
            distanceToTarget = tagDistance + outtakeXOffsetIn;
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

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        turret = hardwareMap.get(Servo.class, "turret");
        turret.setDirection(Servo.Direction.REVERSE);
        // Leave the turret free for manual positioning until START.
        if (turret instanceof PwmControl) {
            ((PwmControl) turret).setPwmDisable();
        }
        intake.setDirection(DcMotor.Direction.REVERSE);
        // Keep the motor direction used by TeleOp. Shooter shares this transfer motor.
        // transfer.setDirection(DcMotor.Direction.REVERSE);

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
                AngleUnit.RADIANS, 3 * Math.PI / 2));

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

    private void drive(double targetX, double targetY, double targetHeading, boolean intakeHelper, boolean outtakeHelper, int tag){

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
            pinpoint.update();
            Pose2D currentPosition = pinpoint.getPosition();
            updateAim(currentPosition, tag, false);
            updateShooter(tag);
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
                aim.addTelemetryAuto(telemetry);
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
        drive(parkXMM, parkYMM, 0, true, false, secondTag);
    }

    private Pose fieldPose(Pose2D localPose) {
        double tileIn = DistanceUnit.INCH.fromMm(tileLength);
        // In the blue field frame, the goal is (1, 1.5) tiles from the starting robot center.
        double x = FieldMap.PIVOT_X - tileIn + localPose.getY(DistanceUnit.INCH);
        double y = FieldMap.PIVOT_Y + FieldMap.CELL_AIM_FROM_PIVOT_IN
                - 1.5 * tileIn - localPose.getX(DistanceUnit.INCH);
        double heading = localPose.getHeading(AngleUnit.RADIANS) - Math.PI / 2;
        return new Pose(x, y, heading);
    }

    private void updateAim(Pose2D localPose, int tagID, boolean shooting) {
        int cell = FieldMap.clusterOf(Alliance.BLUE.toBlueTag(tagID));
        aim.updateFromPoseAuto(fieldPose(localPose), cell, shooting);
    }

    private void runShooter(int tagID) {
        outtakeOn = true;
        shooter.stopFeeder();

        ElapsedTime flywheelTimer = new ElapsedTime();
        ElapsedTime aimSettle = new ElapsedTime();
        while (opModeIsActive() && flywheelTimer.seconds() < 2.0) {
            pinpoint.update();
            updateAim(pinpoint.getPosition(), tagID, true);
            updateShooter(tagID);
            if (!aim.isAimed()) {
                aimSettle.reset();
            }
            if (aimSettle.milliseconds() >= 250
                    && Math.abs(shooter.getVelocity() - shooter.getGoal()) <= 50) {
                break;
            }
            aim.addTelemetryAuto(telemetry);
            shooter.addTelemetry(telemetry);
            telemetry.update();
            sleep(50);
        }

        shooter.startFeeding();
        ElapsedTime shootTimer = new ElapsedTime();
        double shootSeconds = 5;

        try {
            while (opModeIsActive() && shootTimer.seconds() < shootSeconds) {
                pinpoint.update();
                updateAim(pinpoint.getPosition(), tagID, true);
                updateShooter(tagID);
                boolean aimReady = aim.isAimed();
                intake.setPower(aimReady ? -0.5 : 0);
                shooter.runFeeder(aimReady);
                telemetry.addData("Shot aim", aimReady ? "Ready" : "Waiting for turret");
                aim.addTelemetryAuto(telemetry);
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
