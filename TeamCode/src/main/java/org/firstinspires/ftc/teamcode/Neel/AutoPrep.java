package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Auto Prep", group = "Auto")
public class AutoPrep extends LinearOpMode {

    private DcMotorEx outtake;
    private CRServo intake;
    private CRServo storageWheel;
    private Limelight3A limelight;
    public static final int TARGET_TAG_ID = 20;
    public static int IDLE_VELOCITY = 600;

    public static double NEW_P = 7.0;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 15.0;

    private ElapsedTime storageTimer = new ElapsedTime();
    int goalVelocity = 0;
    double range = 0.02;
    double distanceToTarget = 0;
    double angleToTarget = 0;
    double currentVelocity = 0;
    double currentPower = 0;
    private int frontLeftTarget = 0;
    private int backLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backRightTarget = 0;

    boolean isShooting = false;
    boolean isAtGoalVelocity = false;
    boolean shooterNeedsReset = false;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private IMU imu = null;

    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.3;
    static final double HEADING_THRESHOLD = 1.0;
    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeMotors();
        initializeLimelight();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.addData("DistanceToTarget", getDistanceToTag(TARGET_TAG_ID));
        telemetry.update();

        while (opModeInInit()) {
            telemetry.addData(">", "Imu int = %4.1f", getHeading());
            telemetry.addData("> ticks", frontLeftMotor.getCurrentPosition());
            telemetry.addData("DistanceToTarget", getDistanceToTag(TARGET_TAG_ID));
            telemetry.addData("AngleToTarget", getAngleToTag(TARGET_TAG_ID));
            telemetry.update();
        }

        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        DRIVE_SPEED = 0.5;
        outtake.setPower(0.4);

        imu.resetYaw();
        shooterNeedsReset = false;
        distanceToTarget = getDistanceToTag(TARGET_TAG_ID);

        intake.setPower(-1);
        checkIfShooting();
        setGoalVelocity();
        runOuttakeMotor();

        resetRuntime();
        double runtime = getRuntime();
        while (runtime <= 6) {
            if (isStopRequested() || !opModeIsActive()) return;
            distanceToTarget = getDistanceToTag(TARGET_TAG_ID);
            setGoalVelocity();
            runOuttakeMotor();
            doShooting();
            runtime = getRuntime();

            telemetry.addData("Motor running", runtime);
            telemetry.update();
        }

        storageWheel.setPower(0);
        outtake.setPower(0);
        isShooting = false;

        imu.resetYaw();
        driveStraight(DRIVE_SPEED, 25, 0);

        imu.resetYaw();
        turnToHeading(TURN_SPEED, 66);
        holdHeading(TURN_SPEED, 66, 0.5);
        DRIVE_SPEED = 0.2;
        imu.resetYaw();
        driveStraight(DRIVE_SPEED, 29, 0.0);
        imu.resetYaw();
        DRIVE_SPEED = 0.5;
        outtake.setPower(0.5);

        driveStraight(DRIVE_SPEED, -26, 0.5);
        imu.resetYaw();
        holdHeading(TURN_SPEED, -66, 0.5);
        turnToHeading(TURN_SPEED, -66);

        imu.resetYaw();
        driveStraight(DRIVE_SPEED, -18, 0);
        intake.setPower(-1);

        shooterNeedsReset = false;
        checkIfShooting();
        distanceToTarget = getDistanceToTag(TARGET_TAG_ID);
        setGoalVelocity();
        runOuttakeMotor();

        resetRuntime();
        runtime = getRuntime();
        while (runtime <= 8) {
            if (!opModeIsActive() || isStopRequested()) return;
            setGoalVelocity();
            runOuttakeMotor();
            doShooting();
            runtime = getRuntime();
            telemetry.addData("First loop", runtime);
            telemetry.update();
        }

        storageWheel.setPower(0);
        outtake.setPower(0);
        driveStraight(DRIVE_SPEED, 18, 0);

        limelight.stop();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void initializeMotors() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        intake = hardwareMap.crservo.get("intake");
        storageWheel = hardwareMap.crservo.get("storageWheel");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
    }

    private void initializeLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        ResetEncoders();
        if (opModeIsActive()) {
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeftMotor.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightMotor.getCurrentPosition() + moveCounts;
            backRightTarget = backRightMotor.getCurrentPosition() + moveCounts;

            frontLeftMotor.setTargetPosition(frontLeftTarget);
            backLeftMotor.setTargetPosition(backLeftTarget);
            frontRightMotor.setTargetPosition(frontRightTarget);
            backRightMotor.setTargetPosition(backRightTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (opModeIsActive() &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                if (distance < 0)
                    turnSpeed *= -1.0;
                moveRobot(driveSpeed, turnSpeed);
                sendTelemetry(true);
            }

            moveRobot(0, 0);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    private void ResetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;
        headingError = targetHeading - getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        backLeftMotor.setPower(leftSpeed);
        backRightMotor.setPower(rightSpeed);
    }

    private void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", frontLeftTarget, frontRightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    private void doShooting() {
        if (isAtGoalVelocity) {
            storageTimer.reset();
            storageWheel.setPower(-1);
            while (opModeIsActive() && storageTimer.milliseconds() < 4000 && !shooterNeedsReset) {
                runOuttakeMotor();
                telemetry.addData("velocity", goalVelocity);
                telemetry.addData("curPower", currentPower);
                telemetry.update();
            }
            shooterNeedsReset = true;
        }
    }

    private void checkIfShooting() {
        isShooting = true;
    }

    private void setGoalVelocity() {
        int tempVelocity = goalVelocity;
        if (distanceToTarget > 40 && distanceToTarget < 140) {
            tempVelocity = (int) (693.198761 + 1191.999926 * (1.0 - Math.exp(-0.007992 * distanceToTarget)) + 25);
        }

        if (isShooting && !shooterNeedsReset) {
            goalVelocity = tempVelocity;
        }
    }

    private void runOuttakeMotor() {
        currentVelocity = outtake.getVelocity();
        currentPower = outtake.getPower();
        telemetry.addData("curVelocity", currentVelocity);
        telemetry.addData("curPower", currentPower);

        double minRange = goalVelocity - (goalVelocity * range);
        double maxRange = goalVelocity;
        telemetry.addData("minRange", minRange);
        telemetry.addData("maxRange", maxRange);

        isAtGoalVelocity = (currentVelocity <= maxRange) && (currentVelocity > minRange);
        telemetry.addData("isAtGoalVelocity", isAtGoalVelocity);
        if (isAtGoalVelocity) {
            return;
        }

        outtake.setVelocity(goalVelocity);
    }

    private LLResultTypes.FiducialResult findTag(int tagId) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == tagId) {
                return tag;
            }
        }
        return null;
    }

    private int getDistanceToTag(int tagId) {
        LLResultTypes.FiducialResult tag = findTag(tagId);
        if (tag == null) {
            return 0;
        }
        Position pos = tag.getTargetPoseCameraSpace().getPosition();
        double x = pos.unit.toInches(pos.x);
        double y = pos.unit.toInches(pos.y);
        double z = pos.unit.toInches(pos.z);
        return (int) Math.sqrt(x * x + y * y + z * z);
    }

    private int getAngleToTag(int tagId) {
        LLResultTypes.FiducialResult tag = findTag(tagId);
        if (tag == null) {
            return 0;
        }
        return (int) -tag.getTargetXDegrees();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}