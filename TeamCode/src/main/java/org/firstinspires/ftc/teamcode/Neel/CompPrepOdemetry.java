package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "Comp Prep Odometry", group = "Glitch")
public class CompPrepOdemetry extends LinearOpMode {
    public static double NEW_P = 7.0;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 15.0;

    public static double AIM_KP = 0.02;
    public static double AIM_KD = 0.0;
    public static double AIM_MIN_TURN = 0.0;
    public static double AIM_MAX_TURN = 0.4;
    public static double AIM_TOLERANCE = 1.0;
    public static double AIM_OFFSET_DEG = 0.0;

    public static String PINPOINT_NAME = "pinpoint";
    public static double X_POD_OFFSET = 0.0;
    public static double Y_POD_OFFSET = 0.0;
    public static GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static double CUSTOM_TICKS_PER_INCH = 0.0;

    public static double FIELD_SIZE = 144.0;

    public static double BLUE_CELL_A_X = 0.0, BLUE_CELL_A_Y = 0.0;
    public static double BLUE_CELL_B_X = 0.0, BLUE_CELL_B_Y = 0.0;
    public static double RED_CELL_A_X = 0.0, RED_CELL_A_Y = 0.0;
    public static double RED_CELL_B_X = 0.0, RED_CELL_B_Y = 0.0;

    public static double BLUE_START_X = 0.0, BLUE_START_Y = 0.0, BLUE_START_HEADING_DEG = 0.0;
    public static double RED_START_X = 0.0, RED_START_Y = 0.0, RED_START_HEADING_DEG = 0.0;

    public static double BLUE_RELOC_X = 0.0, BLUE_RELOC_Y = 0.0, BLUE_RELOC_HEADING_DEG = 0.0;
    public static double RED_RELOC_X = 0.0, RED_RELOC_Y = 0.0, RED_RELOC_HEADING_DEG = 0.0;

    public static double SHOOTER_FORWARD = 0.0;
    public static double SHOOTER_LEFT = 0.0;
    public static double SHOOTER_DIRECTION_DEG = 0.0;

    public static double HEIGHT_DIFF = 0.0;
    public static double MIN_DIST = 40.0;
    public static double MAX_DIST = 140.0;

    public static boolean BLEND_ZONES = false;

    public enum RedMapping { SAME, MIRROR_X, MIRROR_Y, ROTATE_180 }
    public static RedMapping RED_MAPPING = RedMapping.MIRROR_X;

    public static double VEL_TRIM_STEP = 20.0;
    public static double AIM_TRIM_STEP = 0.5;
    public static String TRIM_FILE = "/sdcard/FIRST/zone_trims.csv";

    private static final double[] D = {1088.2069, 0.4127235, 1.4620166};

    private static double[] f(double a, double b, double p) {
        return new double[]{a, b, p};
    }

    public static double[][][] ZONE_FORMULAS = {
            {D, D, D, D, D, D},
            {D, D, D, D, D, D},
            {D, D, D, D, D, D},
            {D, D, D, D, D, D},
            {D, D, D, D, D, D},
            {D, D, D, D, D, D},
    };

    public static class PoseStorage {
        public static boolean hasPose = false;
        public static double x = 0, y = 0, headingRad = 0;

        public static void save(double x, double y, double headingRad) {
            PoseStorage.x = x;
            PoseStorage.y = y;
            PoseStorage.headingRad = headingRad;
            PoseStorage.hasPose = true;
        }
    }

    private enum Alliance { BLUE, RED }

    private CRServo transfer;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx outtake;
    private DcMotorEx intake;
    private CRServo rightServo;
    private CRServo leftServo;
    private GoBildaPinpointDriver pinpoint;

    int goalVelocity = 0;
    double range = 0.02;
    double minRange = 0;
    double maxRange = 0;

    double distanceToTarget = 0;
    double floorDistance = 0;
    double angleToTarget = 0;
    double currentVelocity = 0;

    boolean isShooting = false;
    boolean isAtGoalVelocity = false;
    boolean shooterNeedsReset = false;
    boolean isAimedAtTarget = false;

    private Alliance alliance = Alliance.BLUE;
    private boolean keepPoseFromAuto = false;
    private boolean targetCellB = false;

    private double robotX = 0, robotY = 0, robotHeading = 0;
    private double headingVelDeg = 0;
    private boolean poseValid = true;

    private boolean targetConfigured = false;
    private boolean shotValid = false;
    private double targetHeading = 0;
    private double formulaVelocity = 0;
    private int zoneRow = 0, zoneCol = 0;

    private double[][] velTrim;
    private double[][] aimTrim;
    private boolean trimsDirty = false;
    private String trimStatus = "";

    private final boolean[] prevButton = new boolean[16];
    private static final int G1_X = 0, G1_B = 1, G1_Y = 2, G1_BACK = 3, G1_LB = 4,
            G2_UP = 5, G2_DOWN = 6, G2_LEFT = 7, G2_RIGHT = 8, G2_A = 9, G2_X = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();
        initializePinpoint();
        initializeZones();

        while (opModeInInit()) {
            pinpoint.update();
            readPose();
            if (pressed(G1_X, gamepad1.x)) alliance = Alliance.BLUE;
            if (pressed(G1_B, gamepad1.b)) alliance = Alliance.RED;
            if (pressed(G1_Y, gamepad1.y)) keepPoseFromAuto = !keepPoseFromAuto;

            telemetry.addLine("gamepad1: X = BLUE, B = RED, Y = toggle keep pose from auto");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Start pose", keepPoseFromAuto
                    ? (PoseStorage.hasPose ? "from auto (PoseStorage)" : "keep Pinpoint's current pose")
                    : "START pose for " + alliance);
            telemetry.addData("Pinpoint pose now", "x %.1f  y %.1f  h %.1f°",
                    robotX, robotY, Math.toDegrees(robotHeading));
            telemetry.addLine("Check: turning the robot LEFT by hand must make h go UP.");
            addConfigWarnings();
            telemetry.addData("Zone trims", trimStatus);
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        if (keepPoseFromAuto) {
            if (PoseStorage.hasPose) {
                setPose(PoseStorage.x, PoseStorage.y, PoseStorage.headingRad);
            }
        } else if (alliance == Alliance.BLUE) {
            if (isSet(BLUE_START_X, BLUE_START_Y)) {
                setPose(BLUE_START_X, BLUE_START_Y, Math.toRadians(BLUE_START_HEADING_DEG));
            }
        } else {
            if (isSet(RED_START_X, RED_START_Y)) {
                setPose(RED_START_X, RED_START_Y, Math.toRadians(RED_START_HEADING_DEG));
            }
        }

        while (opModeIsActive()) {
            pinpoint.update();
            readPose();
            handleOdometryButtons();
            computeShot();

            RunIntakemotor();
            doDriving();
            setGoalVelocity();
            runOuttakeMotor();
            checkIfShooting();
            checkToResetState();
            twoSideMotors();

            addTelemetry();
            telemetry.update();
        }

        outtake.setPower(0);
        intake.setPower(0);
        if (trimsDirty) saveTrims();
    }

    private void computeShot() {
        double tx, ty;
        if (alliance == Alliance.BLUE) {
            tx = targetCellB ? BLUE_CELL_B_X : BLUE_CELL_A_X;
            ty = targetCellB ? BLUE_CELL_B_Y : BLUE_CELL_A_Y;
        } else {
            tx = targetCellB ? RED_CELL_B_X : RED_CELL_A_X;
            ty = targetCellB ? RED_CELL_B_Y : RED_CELL_A_Y;
        }
        targetConfigured = !(tx == 0.0 && ty == 0.0);
        shotValid = targetConfigured && poseValid;
        if (!poseValid) return;

        double cos = Math.cos(robotHeading), sin = Math.sin(robotHeading);
        double shooterX = robotX + SHOOTER_FORWARD * cos - SHOOTER_LEFT * sin;
        double shooterY = robotY + SHOOTER_FORWARD * sin + SHOOTER_LEFT * cos;

        floorDistance = Math.hypot(tx - shooterX, ty - shooterY);
        double d = Math.sqrt(floorDistance * floorDistance + HEIGHT_DIFF * HEIGHT_DIFF);
        distanceToTarget = clamp(d, MIN_DIST, MAX_DIST);

        double[] zp = zoneFramePosition(robotX, robotY);
        int rows = ZONE_FORMULAS.length;
        int cols = ZONE_FORMULAS[0].length;
        zoneRow = zoneIndex(zp[1], FIELD_SIZE, rows);
        zoneCol = zoneIndex(zp[0], FIELD_SIZE, cols);

        double zoneAimTrim;
        if (BLEND_ZONES) {
            double[] out = blendedZoneOutputs(zp[0], zp[1], distanceToTarget);
            formulaVelocity = out[0];
            zoneAimTrim = out[1];
        } else {
            formulaVelocity = zoneVelocity(zoneRow, zoneCol, distanceToTarget);
            zoneAimTrim = aimTrim[zoneRow][zoneCol];
        }

        targetHeading = aimHeading(robotX, robotY, tx, ty,
                SHOOTER_FORWARD, SHOOTER_LEFT, Math.toRadians(SHOOTER_DIRECTION_DEG))
                - Math.toRadians(AIM_OFFSET_DEG + zoneAimTrim);
        angleToTarget = Math.toDegrees(wrap(targetHeading - robotHeading));
    }

    private void setGoalVelocity() {
        int tempVelocity = goalVelocity;
        if (shotValid) {
            tempVelocity = (int) Math.round(formulaVelocity);
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

    private double zoneVelocity(int row, int col, double d) {
        double[] fz = ZONE_FORMULAS[ZONE_FORMULAS.length - 1 - row][col];
        return fz[0] + fz[1] * Math.pow(d, fz[2]) + velTrim[row][col];
    }

    private double[] blendedZoneOutputs(double x, double y, double d) {
        int rows = ZONE_FORMULAS.length;
        int cols = ZONE_FORMULAS[0].length;
        double gx = x / (FIELD_SIZE / cols) - 0.5;
        double gy = y / (FIELD_SIZE / rows) - 0.5;
        int c0raw = (int) Math.floor(gx);
        int r0raw = (int) Math.floor(gy);
        double fx = gx - c0raw;
        double fy = gy - r0raw;
        int c0 = clampInt(c0raw, 0, cols - 1), c1 = clampInt(c0raw + 1, 0, cols - 1);
        int r0 = clampInt(r0raw, 0, rows - 1), r1 = clampInt(r0raw + 1, 0, rows - 1);

        double v = lerp(lerp(zoneVelocity(r0, c0, d), zoneVelocity(r0, c1, d), fx),
                lerp(zoneVelocity(r1, c0, d), zoneVelocity(r1, c1, d), fx), fy);
        double a = lerp(lerp(aimTrim[r0][c0], aimTrim[r0][c1], fx),
                lerp(aimTrim[r1][c0], aimTrim[r1][c1], fx), fy);
        return new double[]{v, a};
    }

    private double[] zoneFramePosition(double x, double y) {
        if (alliance == Alliance.BLUE) return new double[]{x, y};
        switch (RED_MAPPING) {
            case MIRROR_X:
                return new double[]{FIELD_SIZE - x, y};
            case MIRROR_Y:
                return new double[]{x, FIELD_SIZE - y};
            case ROTATE_180:
                return new double[]{FIELD_SIZE - x, FIELD_SIZE - y};
            case SAME:
            default:
                return new double[]{x, y};
        }
    }

    private void doDriving() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        boolean autoAiming = gamepad1.left_trigger > 0 && shotValid;
        if (autoAiming) {
            rx = aimTurnPower(angleToTarget, headingVelDeg);
        }
        isAimedAtTarget = shotValid && Math.abs(angleToTarget) <= AIM_TOLERANCE;
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

    static double aimTurnPower(double errDeg, double headingVelDeg) {
        if (Math.abs(errDeg) <= AIM_TOLERANCE) return 0;
        double turn = -AIM_KP * errDeg + AIM_KD * headingVelDeg;
        if (Math.abs(turn) < AIM_MIN_TURN) turn = -Math.signum(errDeg) * AIM_MIN_TURN;
        return clamp(turn, -AIM_MAX_TURN, AIM_MAX_TURN);
    }

    private void checkToResetState() {
        if (gamepad1.right_bumper) {
            goalVelocity = 0;
            outtake.setPower(0);
            intake.setPower(0);
        }
    }

    private void checkIfShooting() {
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

        telemetry.setMsTransmissionInterval(50);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        PIDFCoefficients pidfValues = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Flywheel PIDF",
                "P=%.6f I=%.6f D=%.6f F=%.6f",
                pidfValues.p, pidfValues.i, pidfValues.d, pidfValues.f);
    }

    private void initializePinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        pinpoint.setOffsets(X_POD_OFFSET, Y_POD_OFFSET, DistanceUnit.INCH);
        if (CUSTOM_TICKS_PER_INCH > 0) {
            pinpoint.setEncoderResolution(CUSTOM_TICKS_PER_INCH, DistanceUnit.INCH);
        } else {
            pinpoint.setEncoderResolution(POD_TYPE);
        }
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);

        pinpoint.recalibrateIMU();
        sleep(500);
        pinpoint.update();
        readPose();
    }

    private void readPose() {
        robotX = pinpoint.getPosX(DistanceUnit.INCH);
        robotY = pinpoint.getPosY(DistanceUnit.INCH);
        robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        headingVelDeg = Math.toDegrees(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        poseValid = !(Double.isNaN(robotX) || Double.isNaN(robotY) || Double.isNaN(robotHeading)
                || Double.isInfinite(robotX) || Double.isInfinite(robotY) || Double.isInfinite(robotHeading));
        if (Double.isNaN(headingVelDeg) || Double.isInfinite(headingVelDeg)) headingVelDeg = 0;
    }

    private void setPose(double x, double y, double headingRad) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRad));
        robotX = x;
        robotY = y;
        robotHeading = headingRad;
    }

    private void handleOdometryButtons() {
        boolean reloc = pressed(G1_BACK, gamepad1.back);
        boolean toggleCell = pressed(G1_LB, gamepad1.left_bumper);
        boolean velUp = pressed(G2_UP, gamepad2.dpad_up);
        boolean velDown = pressed(G2_DOWN, gamepad2.dpad_down);
        boolean aimLeft = pressed(G2_LEFT, gamepad2.dpad_left);
        boolean aimRight = pressed(G2_RIGHT, gamepad2.dpad_right);
        boolean clearZone = pressed(G2_A, gamepad2.a);
        boolean save = pressed(G2_X, gamepad2.x);

        if (reloc) {
            if (alliance == Alliance.BLUE && isSet(BLUE_RELOC_X, BLUE_RELOC_Y)) {
                setPose(BLUE_RELOC_X, BLUE_RELOC_Y, Math.toRadians(BLUE_RELOC_HEADING_DEG));
            } else if (alliance == Alliance.RED && isSet(RED_RELOC_X, RED_RELOC_Y)) {
                setPose(RED_RELOC_X, RED_RELOC_Y, Math.toRadians(RED_RELOC_HEADING_DEG));
            }
        }
        if (toggleCell) targetCellB = !targetCellB;
        if (gamepad2.left_bumper) targetCellB = false;
        if (gamepad2.right_bumper) targetCellB = true;

        if (velUp) { velTrim[zoneRow][zoneCol] += VEL_TRIM_STEP; trimsDirty = true; }
        if (velDown) { velTrim[zoneRow][zoneCol] -= VEL_TRIM_STEP; trimsDirty = true; }
        if (aimRight) { aimTrim[zoneRow][zoneCol] += AIM_TRIM_STEP; trimsDirty = true; }
        if (aimLeft) { aimTrim[zoneRow][zoneCol] -= AIM_TRIM_STEP; trimsDirty = true; }
        if (clearZone) { velTrim[zoneRow][zoneCol] = 0; aimTrim[zoneRow][zoneCol] = 0; trimsDirty = true; }
        if (save) saveTrims();
    }

    private void initializeZones() {
        int rows = ZONE_FORMULAS.length;
        int cols = ZONE_FORMULAS[0].length;
        velTrim = new double[rows][cols];
        aimTrim = new double[rows][cols];
        int loaded = loadTrims();
        if (trimStatus.isEmpty()) {
            trimStatus = loaded > 0 ? ("loaded " + loaded + " zones from " + TRIM_FILE) : "no saved trims yet";
        }
    }

    private int loadTrims() {
        File file = new File(TRIM_FILE);
        if (!file.exists()) return 0;
        int rows = velTrim.length, cols = velTrim[0].length, n = 0;
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                String[] p = line.split(",");
                if (p.length < 4) continue;
                try {
                    int r = Integer.parseInt(p[0].trim());
                    int c = Integer.parseInt(p[1].trim());
                    double v = Double.parseDouble(p[2].trim());
                    double a = Double.parseDouble(p[3].trim());
                    if (r >= 0 && r < rows && c >= 0 && c < cols) {
                        velTrim[r][c] = v;
                        aimTrim[r][c] = a;
                        n++;
                    }
                } catch (NumberFormatException ignored) {
                }
            }
        } catch (IOException e) {
            trimStatus = "LOAD FAILED: " + e.getMessage();
        }
        return n;
    }

    private void saveTrims() {
        File file = new File(TRIM_FILE);
        try {
            File parent = file.getParentFile();
            if (parent != null) parent.mkdirs();
            try (FileWriter w = new FileWriter(file, false)) {
                w.write("# row(from bottom),column(from left),velocityTrim,aimTrimDeg  (blue frame)\n");
                int n = 0;
                for (int r = 0; r < velTrim.length; r++) {
                    for (int c = 0; c < velTrim[0].length; c++) {
                        if (velTrim[r][c] != 0 || aimTrim[r][c] != 0) {
                            w.write(String.format(Locale.US, "%d,%d,%.1f,%.2f\n", r, c, velTrim[r][c], aimTrim[r][c]));
                            n++;
                        }
                    }
                }
                trimStatus = "saved " + n + " zones";
            }
            trimsDirty = false;
        } catch (IOException e) {
            trimStatus = "SAVE FAILED: " + e.getMessage();
        }
    }
    private void addTelemetry() {
        addConfigWarnings();
        telemetry.addData("pose", "x %.1f  y %.1f  h %.1f°", robotX, robotY, Math.toDegrees(robotHeading));
        telemetry.addData("target", "%s cell %s", alliance, targetCellB ? "B" : "A");
        telemetry.addData("zone", "row %d (from bottom), col %d (from left)%s",
                zoneRow, zoneCol, BLEND_ZONES ? "  [blended]" : "");
        telemetry.addData("floor distance", "%.1f in", floorDistance);
        telemetry.addData("distance", "%.1f in", distanceToTarget);
        telemetry.addData("angle", "%.1f°", angleToTarget);
        telemetry.addData("zone trim", "vel %+.0f   aim %+.1f°",
                velTrim[zoneRow][zoneCol], aimTrim[zoneRow][zoneCol]);
        telemetry.addData("trims", trimStatus + (trimsDirty ? "  (unsaved, gamepad2 X)" : ""));

        for (int r = velTrim.length - 1; r >= 0; r--) {
            StringBuilder sb = new StringBuilder();
            for (int c = 0; c < velTrim[0].length; c++) {
                String cell = String.format(Locale.US, "%+4.0f", velTrim[r][c]);
                sb.append(r == zoneRow && c == zoneCol ? "[" + cell + "]" : " " + cell + " ");
            }
            telemetry.addLine(sb.toString());
        }
    }

    private void addConfigWarnings() {
        double ax = alliance == Alliance.BLUE ? BLUE_CELL_A_X : RED_CELL_A_X;
        double ay = alliance == Alliance.BLUE ? BLUE_CELL_A_Y : RED_CELL_A_Y;
        if (ax == 0.0 && ay == 0.0) {
            telemetry.addLine("!! Set the " + alliance + " CELL coordinates - auto-aim is OFF until you do");
        }
        if (X_POD_OFFSET == 0.0 && Y_POD_OFFSET == 0.0) {
            telemetry.addLine("!! Pod offsets are 0 - copy them from your Pedro Constants");
        }
        boolean startSet = alliance == Alliance.BLUE ? isSet(BLUE_START_X, BLUE_START_Y) : isSet(RED_START_X, RED_START_Y);
        if (!keepPoseFromAuto && !startSet) {
            telemetry.addLine("!! " + alliance + " START pose not set - pose will NOT be reset at Play");
        }
        if (!poseValid) {
            telemetry.addLine("!! Pinpoint reading is invalid - check wiring / config name '" + PINPOINT_NAME + "'");
        }
    }

    private static boolean isSet(double x, double y) {
        return !(x == 0.0 && y == 0.0);
    }

    static double aimHeading(double robotX, double robotY, double targetX, double targetY,
                             double shooterForward, double shooterLeft, double shooterDirRad) {
        double vx = targetX - robotX;
        double vy = targetY - robotY;
        double dist = Math.hypot(vx, vy);
        double bearing = Math.atan2(vy, vx);

        double lateral = shooterLeft * Math.cos(shooterDirRad) - shooterForward * Math.sin(shooterDirRad);
        double correction = 0;
        if (dist > Math.abs(lateral) + 1e-6) {
            correction = Math.asin(lateral / dist);
        }
        return wrap(bearing - shooterDirRad - correction);
    }

    static int zoneIndex(double coord, double fieldSize, int count) {
        int i = (int) Math.floor(coord / (fieldSize / count));
        return clampInt(i, 0, count - 1);
    }

    static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private boolean pressed(int id, boolean now) {
        boolean wasPressed = now && !prevButton[id];
        prevButton[id] = now;
        return wasPressed;
    }
}