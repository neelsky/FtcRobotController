package org.firstinspires.ftc.teamcode.Neel;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Turret Test", group = "Glitch")
public class TurretTest extends LinearOpMode {
    public static double SMALL_STEP = 0.01;
    public static double BIG_STEP = 0.05;

    @Override
    public void runOpMode() {
        Servo turret = hardwareMap.get(Servo.class, "turret");
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                Constants.localizerConstants.hardwareMapName);
        double position = OdometryTurretShooting.TURRET_ZERO_POS;
        String check = "straight back";
        boolean leftWas = false, rightWas = false, lbWas = false, rbWas = false;

        telemetry.addLine("Press START. The turret goes straight back (away from the intake).");
        telemetry.update();
        waitForStart();
        pinpoint.update();
        Pose start = PoseConverter.pose2DToPose(pinpoint.getPosition(), PedroCoordinates.INSTANCE);

        while (opModeIsActive()) {
            if (gamepad1.dpad_right && !rightWas) {
                position += SMALL_STEP;
                check = "manual";
            }
            if (gamepad1.dpad_left && !leftWas) {
                position -= SMALL_STEP;
                check = "manual";
            }
            if (gamepad1.right_bumper && !rbWas) {
                position += BIG_STEP;
                check = "manual";
            }
            if (gamepad1.left_bumper && !lbWas) {
                position -= BIG_STEP;
                check = "manual";
            }
            rightWas = gamepad1.dpad_right;
            leftWas = gamepad1.dpad_left;
            rbWas = gamepad1.right_bumper;
            lbWas = gamepad1.left_bumper;

            if (gamepad1.y) {
                position = positionFor(0);
                check = "straight back (away from the intake)";
            }
            if (gamepad1.x) {
                position = positionFor(90);
                check = "the robot's LEFT side";
            }
            if (gamepad1.b) {
                position = positionFor(-90);
                check = "the robot's RIGHT side";
            }
            if (gamepad1.a) {
                position = positionFor(OdometryTurretShooting.TURRET_LIMIT_DEG);

            }

            position = Math.max(0, Math.min(1, position));
            turret.setPosition(position);

            pinpoint.update();
            Pose now = PoseConverter.pose2DToPose(pinpoint.getPosition(), PedroCoordinates.INSTANCE);
            double dx = now.getX() - start.getX();
            double dy = now.getY() - start.getY();
            double h0 = start.getHeading();
            double towardIntake = dx * Math.cos(h0) + dy * Math.sin(h0);
            double turned = FieldMap.wrapDegrees(Math.toDegrees(now.getHeading() - h0));

            telemetry.addData("servo position", "%.3f", position);
            telemetry.addData("turret should point at", check);
            telemetry.addLine("Y = straight back, X = robot's LEFT side, B = robot's RIGHT side, A = at the intake");
            telemetry.addLine("Left/right = robot's left/right, as if you were sitting in it facing the intake.");
            telemetry.addLine("Dpad left/right = 0.01 steps, bumpers = 0.05 steps");
            telemetry.addLine("");
            telemetry.addData("robot turned", "%+.1f deg   (turn the robot LEFT by hand: must go UP)", turned);
            telemetry.addData("moved toward intake", "%+.1f in   (push the robot toward the intake: must go UP)", towardIntake);
            telemetry.update();
        }
    }

    private double positionFor(double angleDeg) {
        double units = angleDeg / OdometryTurretShooting.TURRET_DEGREES_PER_UNIT;
        return OdometryTurretShooting.TURRET_HIGHER_IS_LEFT
                ? OdometryTurretShooting.TURRET_ZERO_POS + units
                : OdometryTurretShooting.TURRET_ZERO_POS - units;
    }
}
