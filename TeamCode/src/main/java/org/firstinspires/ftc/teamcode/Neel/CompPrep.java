package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Comp Prep", group = "Glitch")
public class CompPrep extends LinearOpMode {

    public static double SHOOTER_KF = 0.00042;
    public static int DEFAULT_VELOCITY = 900;
    public static int MAX_VELOCITY = 2200;

    public static double READY_AT = 0.97;
    public static double DROP_FROM_PEAK = 0.03;
    public static double MIN_PAUSE_MS = 150;
    public static double MAX_PAUSE_MS = 500;
    public static double SPIN_UP_MS = 1200;
    public static double TRANSFER_POWER = 1.0;

    public static double TAG_HOLD_MS = 300;

    public static double REAL_TAG_MM = 82.55;
    public static double LIMELIGHT_MARKER_MM = 101.6;
    public static double CELL_DEPTH_OFFSET_IN = 0;

    public static double CAMERA_RIGHT_OF_CENTER_IN = 5.0 / 2.54;
    public static double CAMERA_FORWARD_OF_CENTER_IN = 0;
    public static double AIM_TRIM_DEG = 0;

    public static double AIM_KP = 0.015;
    public static double AIM_KS = 0.06;
    public static double AIM_MAX_TURN = 0.5;
    public static double AIM_TOLERANCE = 0.75;
    public static double AIM_UPDATE_MAX_RATE = 5;
    public static double AIM_STILL_MS = 120;
    private final ElapsedTime stillTimer = new ElapsedTime();

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
    private IMU imu;

    private final ElapsedTime lastSeenTimer = new ElapsedTime();
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private final ElapsedTime fullPowerTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();

    boolean tagVisible = false;
    boolean freshFrame = false;
    double lastFrameSignature = Double.NaN;
    int tagId = -1;
    int tagsUsed = 0;
    boolean hasCameraMiddle = false;
    double cameraMiddleX = 0;
    String aimingAt = "nothing";
    double distanceToTarget = -1;
    double angleToTarget = 0;

    boolean hasFaceAngle = false;
    int faceCluster = -1;
    double faceAngleWorld = 0;
    private final ElapsedTime faceTimer = new ElapsedTime();
    public static double FACE_MEMORY_MS = 3000;

    boolean hasAimHeading = false;
    double aimHeading = 0;

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
        imu.resetYaw();
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
        List<LLResultTypes.FiducialResult> seen = new ArrayList<>();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (isTargetTag(tag.getFiducialId())) {
                    seen.add(tag);
                }
            }
        }

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult tag : seen) {
            if (best == null || tag.getTargetArea() > best.getTargetArea()) {
                best = tag;
            }
        }

        freshFrame = false;

        if (best != null) {
            tagId = best.getFiducialId();
            double[] p = tagPositionInches(best);

            if (p[2] > 1) {
                double[] middle = findClusterMiddle(best, p, seen);
                cameraMiddleX = middle[0];
                hasCameraMiddle = true;
                aimingAt = "CELL CENTER (" + (tagsUsed == 2 ? "2 tags" : "1 tag") + ")";
                double xRobot = middle[0] + CAMERA_RIGHT_OF_CENTER_IN;
                double zRobot = middle[2] + CAMERA_FORWARD_OF_CENTER_IN;
                distanceToTarget = Math.sqrt(middle[0] * middle[0] + middle[1] * middle[1] + middle[2] * middle[2]);
                angleToTarget = Math.toDegrees(Math.atan2(xRobot, zRobot)) + AIM_TRIM_DEG;
            } else {
                hasCameraMiddle = false;
                LLResultTypes.FiducialResult partner = null;
                for (LLResultTypes.FiducialResult tag : seen) {
                    if (tag != best && clusterOf(tag.getFiducialId()) == clusterOf(tagId)
                            && getTagOffsetInches(tag.getFiducialId()) != getTagOffsetInches(tagId)) {
                        partner = tag;
                        break;
                    }
                }
                if (partner != null) {
                    double bestOffset = getTagOffsetInches(tagId);
                    double perInch = (best.getTargetXDegrees() - partner.getTargetXDegrees())
                            / (getTagOffsetInches(partner.getFiducialId()) - bestOffset);
                    angleToTarget = best.getTargetXDegrees() + bestOffset * perInch + AIM_TRIM_DEG;
                    tagsUsed = 2;
                    aimingAt = "CELL CENTER (no 3D, 2 tags)";
                } else {
                    tagsUsed = 0;
                    angleToTarget = best.getTargetXDegrees() + AIM_TRIM_DEG;
                    aimingAt = "TAG ONLY: turn on Full 3D";
                }
            }

            double signature = best.getTargetXDegrees() * 1000 + best.getTargetArea();
            freshFrame = signature != lastFrameSignature;
            lastFrameSignature = signature;

            tagVisible = true;
            lastSeenTimer.reset();
        } else if (lastSeenTimer.milliseconds() > TAG_HOLD_MS) {
            tagVisible = false;
            tagId = -1;
            tagsUsed = 0;
            distanceToTarget = -1;
        }

        if (!tagVisible) {
            aimingAt = "nothing";
        }
        telemetry.addData("tag", tagVisible ? tagId + " (" + tagsUsed + " used)" : "not seen");
        telemetry.addData("aimingAt", aimingAt);
        telemetry.addData("distance", "%.1f in", distanceToTarget);
        telemetry.addData("angle", "%.2f°", angleToTarget);
    }

    private double[] findClusterMiddle(LLResultTypes.FiducialResult best, double[] p,
                                       List<LLResultTypes.FiducialResult> seen) {
        double bestOffset = getTagOffsetInches(best.getFiducialId());

        LLResultTypes.FiducialResult partner = null;
        double[] partnerPos = null;
        double widestGap = 0;
        for (LLResultTypes.FiducialResult tag : seen) {
            if (tag == best || clusterOf(tag.getFiducialId()) != clusterOf(best.getFiducialId())) {
                continue;
            }
            double gap = Math.abs(getTagOffsetInches(tag.getFiducialId()) - bestOffset);
            double[] q = tagPositionInches(tag);
            if (q[2] > 1 && gap > widestGap) {
                partner = tag;
                partnerPos = q;
                widestGap = gap;
            }
        }

        int cluster = clusterOf(best.getFiducialId());
        double heading = getHeading();
        double[] right = {1, 0, 0};
        if (partner != null) {
            double offsetDiff = getTagOffsetInches(partner.getFiducialId()) - bestOffset;
            right = new double[]{
                    (p[0] - partnerPos[0]) / offsetDiff,
                    (p[1] - partnerPos[1]) / offsetDiff,
                    (p[2] - partnerPos[2]) / offsetDiff
            };
            faceAngleWorld = Math.toDegrees(Math.atan2(right[2], right[0])) + heading;
            faceCluster = cluster;
            hasFaceAngle = true;
            faceTimer.reset();
            tagsUsed = 2;
        } else if (hasFaceAngle && faceCluster == cluster && faceTimer.milliseconds() < FACE_MEMORY_MS) {
            double a = Math.toRadians(faceAngleWorld - heading);
            right = new double[]{Math.cos(a), 0, Math.sin(a)};
            tagsUsed = 1;
        } else {
            tagsUsed = 1;
        }

        double[] middle = {
                p[0] + bestOffset * right[0],
                p[1] + bestOffset * right[1],
                p[2] + bestOffset * right[2]
        };

        double[] deeper = {-right[2], 0, right[0]};
        double length = Math.hypot(deeper[0], deeper[2]);
        if (length > 1e-6) {
            if (deeper[2] < 0) {
                deeper[0] = -deeper[0];
                deeper[2] = -deeper[2];
            }
            middle[0] += CELL_DEPTH_OFFSET_IN * deeper[0] / length;
            middle[2] += CELL_DEPTH_OFFSET_IN * deeper[2] / length;
        }

        return middle;
    }

    private double[] tagPositionInches(LLResultTypes.FiducialResult tag) {
        Position pos = tag.getTargetPoseCameraSpace().getPosition();
        double scale = REAL_TAG_MM / LIMELIGHT_MARKER_MM;
        return new double[]{
                pos.unit.toInches(pos.x) * scale,
                pos.unit.toInches(pos.y) * scale,
                pos.unit.toInches(pos.z) * scale
        };
    }

    private boolean isTargetTag(int id) {
        return id >= 38 && id <= 45;
    }

    private int clusterOf(int id) {
        return id <= 41 ? 0 : 1;
    }

    private double getTagOffsetInches(int id) {
        double cm;
        switch (id) {
            case 38: cm = -17; break;
            case 39: cm = -7; break;
            case 40: cm = 7; break;
            case 41: cm = 17; break;
            case 42: cm = 17; break;
            case 43: cm = 7; break;
            case 44: cm = -7; break;
            case 45: cm = -17; break;
            default: cm = 0; break;
        }
        return cm / 2.54;
    }

    private void checkIfShooting() {
        isShooting = gamepad1.right_trigger > 0.1;
    }

    private void setGoalVelocity() {
        if (!isShooting) {
            goalVelocity = 0;
        } else if (tagVisible && distanceToTarget > 32 && distanceToTarget < 114) {
            int fromDistance = (int) Math.round(4.086 * distanceToTarget + 666);
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

        double heading = getHeading();
        double turnRate = Math.abs(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);

        if (turnRate >= AIM_UPDATE_MAX_RATE) {
            stillTimer.reset();
        }
        if (!isShooting) {
            hasAimHeading = false;
        } else if (tagVisible && (!hasAimHeading || stillTimer.milliseconds() >= AIM_STILL_MS)) {
            aimHeading = wrapDegrees(heading - angleToTarget);
            hasAimHeading = true;
        }

        isAimedAtTarget = false;
        if (isShooting && hasAimHeading) {
            double error = wrapDegrees(heading - aimHeading);
            if (Math.abs(error) > AIM_TOLERANCE) {
                rx = AIM_KP * error + Math.signum(error) * AIM_KS;
                rx = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, rx));
            } else {
                rx = 0;
                isAimedAtTarget = true;
            }
            telemetry.addData("aimError", "%.2f°", error);
        }
        telemetry.addData("isAimedAtTarget", isAimedAtTarget);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double wrapDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftServo.setDirection(CRServo.Direction.REVERSE);


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