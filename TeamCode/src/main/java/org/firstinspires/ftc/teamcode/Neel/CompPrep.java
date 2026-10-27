package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "Comp Prep", group = "Glitch")
public class CompPrep extends LinearOpMode {

    public static double SHOOTER_KF = 0.00042;
    public static int DEFAULT_VELOCITY = 1500;
    public static int MAX_VELOCITY = 2200;

    public static double READY_AT = 0.97;
    public static double DROP_FROM_PEAK = 0.03;
    public static double MIN_PAUSE_MS = 150;
    public static double MAX_PAUSE_MS = 500;
    public static double SPIN_UP_MS = 1200;
    public static double TRANSFER_POWER = 1.0;

    public static double TAG_HOLD_MS = 300;

    public static double AIM_KP = 0.02;
    public static double AIM_MAX_TURN = 0.4;
    public static double AIM_TOLERANCE = 2.0;
    public static double AIM_MIN_TURN = 0.08;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx outtake;
    private DcMotorEx intake;
    private CRServo transfer;
    private CRServo rightServo;
    private CRServo leftServo;
    private Limelight3A limelight;

    private final ElapsedTime lastSeenTimer = new ElapsedTime();
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private final ElapsedTime fullPowerTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();

    boolean tagVisible = false;
    int tagId = -1;
    double distanceToTarget = -1;
    double angleToTarget = 0;

    int goalVelocity = 0;
    double currentVelocity = 0;
    double peakVelocity = 0;

    boolean isShooting = false;
    boolean isFeeding = false;
    boolean isAimedAtTarget = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();

        waitForStart();
        lastSeenTimer.reset();
        pauseTimer.reset();

        while (opModeIsActive()) {
            updateVision();
            checkIfShooting();
            setGoalVelocity();
            runOuttakeMotor();
            runTransfer();
            doDriving();
            RunIntakemotor();
            twoSideMotors();
            checkToResetState();
            telemetry.update();
        }

        outtake.setPower(0);
        transfer.setPower(0);
        limelight.stop();
    }

    private void updateVision() {
        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult best = null;

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (isTargetTag(tag.getFiducialId())
                        && (best == null || tag.getTargetArea() > best.getTargetArea())) {
                    best = tag;
                }
            }
        }

        if (best != null) {
            Position pos = best.getTargetPoseCameraSpace().getPosition();
            double x = pos.unit.toInches(pos.x);
            double y = pos.unit.toInches(pos.y);
            double z = pos.unit.toInches(pos.z);
            double dist = Math.sqrt(x * x + y * y + z * z);

            if (dist > 1) {
                distanceToTarget = dist;
            }
            angleToTarget = dist > 100 ? best.getTargetXDegrees() - 2 : best.getTargetXDegrees();
            tagId = best.getFiducialId();
            tagVisible = true;
            lastSeenTimer.reset();
        } else if (lastSeenTimer.milliseconds() > TAG_HOLD_MS) {
            tagVisible = false;
            tagId = -1;
            distanceToTarget = -1;
        }

        telemetry.addData("tag", tagVisible ? String.valueOf(tagId) : "not seen");
        telemetry.addData("distance", "%.1f in", distanceToTarget);
        telemetry.addData("angle", "%.1f°", angleToTarget);
    }

    private boolean isTargetTag(int id) {
        return id >= 38 && id <= 45;
    }

    private void checkIfShooting() {
        isShooting = gamepad1.right_trigger > 0.1;
    }

    private void setGoalVelocity() {
        if (!isShooting) {
            goalVelocity = 0;
        } else if (tagVisible && distanceToTarget > 40 && distanceToTarget < 140) {
            int fromDistance = (int) (930);
            goalVelocity = Math.min(fromDistance, MAX_VELOCITY);
        } else if (goalVelocity == 0) {
            goalVelocity = Math.min(DEFAULT_VELOCITY, MAX_VELOCITY);
        }

        telemetry.addData("goalVelocity", goalVelocity);
    }

    private void runOuttakeMotor() {
        currentVelocity = Math.abs(outtake.getVelocity());

        double power = 0;
        if (goalVelocity > 0) {
            power = currentVelocity < goalVelocity ? 1 : Math.min(1, SHOOTER_KF * goalVelocity);
        }
        outtake.setPower(power);

        if (power < 1) {
            fullPowerTimer.reset();
        }

        telemetry.addData("curVelocity", "%.0f", currentVelocity);
        telemetry.addData("outtakePower", "%.2f", power);
        if (fullPowerTimer.seconds() > 2) {
            telemetry.addLine("Flywheel can't reach goal: lower MAX_VELOCITY");
        }
    }

    private void runTransfer() {
        if (gamepad1.left_trigger > 0.1) {
            isFeeding = false;
            transfer.setPower(TRANSFER_POWER);
            telemetry.addData("transfer", "MANUAL");
            return;
        }

        if (!isShooting || goalVelocity <= 0) {
            isFeeding = false;
            peakVelocity = 0;
            spinUpTimer.reset();
        } else if (isFeeding) {
            peakVelocity = Math.max(peakVelocity, currentVelocity);
            boolean dropped = currentVelocity < peakVelocity * (1 - DROP_FROM_PEAK);
            if (dropped) {
                isFeeding = false;
                pauseTimer.reset();
            }
        } else {
            boolean atSpeed = currentVelocity >= goalVelocity * READY_AT;
            boolean spunUp = spinUpTimer.milliseconds() >= SPIN_UP_MS;
            boolean pausedLongEnough = pauseTimer.milliseconds() >= MAX_PAUSE_MS;
            boolean ready = atSpeed || (spunUp && pausedLongEnough);
            if (ready && pauseTimer.milliseconds() >= MIN_PAUSE_MS) {
                isFeeding = true;
                peakVelocity = currentVelocity;
            }
        }

        transfer.setPower(isFeeding ? TRANSFER_POWER : 0);
        telemetry.addData("transfer", isFeeding ? "FEEDING" : "waiting");
    }

    private void doDriving() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        boolean autoAiming = isShooting && tagVisible;
        if (autoAiming) {
            if (Math.abs(angleToTarget) > AIM_TOLERANCE) {
                rx = angleToTarget * AIM_KP;
                if (Math.abs(rx) < AIM_MIN_TURN) {
                    rx = Math.signum(rx) * AIM_MIN_TURN;
                }
                rx = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, rx));
            } else {
                rx = 0;
            }
        }
        isAimedAtTarget = tagVisible && Math.abs(angleToTarget) <= AIM_TOLERANCE;
        telemetry.addData("isAimedAtTarget", isAimedAtTarget);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }

    private void twoSideMotors() {
        if (gamepad1.a) {
            leftServo.setPower(-1);
            rightServo.setPower(-1);
        }
        if (gamepad1.b) {
            leftServo.setPower(1);
            rightServo.setPower(1);
        }
        if (gamepad1.y) {
            leftServo.setPower(0);
            rightServo.setPower(0);
        }
    }

    private void RunIntakemotor() {
        if (gamepad1.dpad_up) {
            intake.setPower(0.5);
        }
        if (gamepad1.dpad_down) {
            intake.setPower(-0.5);
        }
        if (gamepad1.dpad_left) {
            intake.setPower(0);
        }
    }

    private void checkToResetState() {
        if (gamepad1.right_bumper) {
            goalVelocity = 0;
            isFeeding = false;
            outtake.setPower(0);
            intake.setPower(0);
            transfer.setPower(0);
        }
    }

    private void initializeMotors() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.crservo.get("transfer");
        leftServo = hardwareMap.crservo.get("leftServo");
        rightServo = hardwareMap.crservo.get("rightServo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftServo.setDirection(CRServo.Direction.REVERSE);
        outtake.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setPower(0);
        transfer.setPower(0);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
    }
}