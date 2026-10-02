package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "Comp Prep", group = "Glitch")
public class CompPrep extends LinearOpMode {
    private static final int TARGET_TAG_ID = 8;   // the tag you aim/shoot at

    public static double NEW_P = 7.0;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 15.0;

    // Auto-aim tuning
    public static double AIM_KP = 0.02;        // turn power per degree off target
    public static double AIM_MAX_TURN = 0.4;   // max turn power while aiming
    public static double AIM_TOLERANCE = 1.0;  // degrees: close enough = aimed

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx outtake;
    private CRServo intake;
    private Limelight3A limelight;

    int goalVelocity = 0;
    double range = 0.02;
    double minRange = 0;
    double maxRange = 0;

    double distanceToTarget = 0;
    double angleToTarget = 0;
    double currentVelocity = 0;

    boolean isShooting = false;
    boolean isAtGoalVelocity = false;
    boolean shooterNeedsReset = false;
    boolean isAimedAtTarget = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();   // runs ONCE, before start

        waitForStart();

        while (opModeIsActive()) {
            RunIntakemotor();
            doDriving();
            setGoalVelocity();
            runOuttakeMotor();
            checkIfShooting();
            checkToResetState();

            telemetry.update();
        }

        limelight.stop();
    }

    private void setGoalVelocity() {
        int tempVelocity = goalVelocity;
        distanceToTarget = getDistanceToTag(TARGET_TAG_ID);
        telemetry.addData("distance", "%.1f in", distanceToTarget);

        if (distanceToTarget > 40 && distanceToTarget < 140) {
            tempVelocity = (int) (941.2069 + 0.4127235 * Math.pow(distanceToTarget, 1.4620166) + 147);
            telemetry.addData("tempVelocity", tempVelocity);
        }

        if (isShooting && !shooterNeedsReset) {
            goalVelocity = tempVelocity;
        } else {
            telemetry.addData("isShooting", isShooting);
            telemetry.addData("shooterNeedsReset", shooterNeedsReset);
        }

        telemetry.addData("goalVelocity", goalVelocity);
    }

    private void checkToResetState() {
        // Try to stop anything that isn't responding to other commands.
        if (gamepad1.right_bumper) {
            goalVelocity = 0;
            outtake.setPower(0);
            intake.setPower(0);
        }
    }

    private void checkIfShooting() {
        // To shoot, hold down the left_trigger.
        if (gamepad1.left_trigger > 0 && !shooterNeedsReset) {
            isShooting = true;
        } else if (gamepad1.left_trigger == 0) {
            isShooting = false;
            shooterNeedsReset = false;
        }
    }

    private void runOuttakeMotor() {
        currentVelocity = outtake.getVelocity();
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

    /** Distance in inches from the camera to the tag, or -1 if it isn't visible. */
    private double getDistanceToTag(int tagId) {
        LLResultTypes.FiducialResult tag = findTag(tagId);
        if (tag == null) {
            return -1;
        }

        Position pos = tag.getTargetPoseCameraSpace().getPosition();
        double x = pos.unit.toInches(pos.x);
        double y = pos.unit.toInches(pos.y);
        double z = pos.unit.toInches(pos.z);
        return Math.sqrt(x * x + y * y + z * z);
    }

    /** Left/right angle to the tag in degrees (0 = centered), or 0 if it isn't visible. */
    private double getAngleToTag(int tagId) {
        LLResultTypes.FiducialResult tag = findTag(tagId);
        if (tag == null) {
            return 0;
        }
        return tag.getTargetXDegrees();
    }

    /** Finds the tag with this ID in the latest Limelight result, or null if not seen. */
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

    private void doDriving() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double dist = getDistanceToTag(TARGET_TAG_ID);
        boolean tagVisible = dist >= 0;

        if (!tagVisible) {
            telemetry.addData("angle", "tag not seen");
        } else {
            if (dist > 100) {
                angleToTarget = getAngleToTag(TARGET_TAG_ID) - 2;
            } else {
                angleToTarget = getAngleToTag(TARGET_TAG_ID);
            }
            telemetry.addData("angle", "%.1f°", angleToTarget);
        }

        // Auto-aim: while holding the shoot trigger and the tag is visible,
        // the robot turns itself toward the tag (replaces the right stick turn).
        boolean autoAiming = gamepad1.left_trigger > 0 && tagVisible;
        if (autoAiming) {
            if (Math.abs(angleToTarget) > AIM_TOLERANCE) {
                rx = angleToTarget * AIM_KP;
                rx = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, rx));
            } else {
                rx = 0;
            }
        }
        isAimedAtTarget = tagVisible && Math.abs(angleToTarget) <= AIM_TOLERANCE;
        telemetry.addData("autoAiming", autoAiming);
        telemetry.addData("isAimedAtTarget", isAimedAtTarget);

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

    private void initializeMotors() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake = hardwareMap.crservo.get("intake");

        // Assign the FIELD (no "Limelight3A" in front), so the other methods can use it
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // setVelocity needs the encoder mode
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        PIDFCoefficients pidfValues = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Flywheel PIDF",
                "P=%.6f I=%.6f D=%.6f F=%.6f",
                pidfValues.p, pidfValues.i, pidfValues.d, pidfValues.f);
        telemetry.addLine("Ready. Press START.");
        telemetry.update();
    }

    private void RunIntakemotor() {
        if (gamepad1.dpad_up) {
            intake.setPower(1);
        }
        if (gamepad1.dpad_down) {
            intake.setPower(-1);
        }
        if (gamepad1.dpad_left) {
            intake.setPower(0);
        }
    }
}