package org.firstinspires.ftc.teamcode.Neel.opmodes.testers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Neel.subsystems.map.FieldMap;
import org.firstinspires.ftc.teamcode.Neel.subsystems.turret.AutoAimTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Test OpMode for {@link AutoAimTurret}.
 * Features:
 * - Mecanum driving via Gamepad 1 joysticks
 * - Automatic turret aiming based on robot odometry and Limelight vision
 * - Buttons (X / B) to reset robot pose to LEFT and RIGHT corners using FieldMapMetric coordinates
 */
@TeleOp(name = "AutoAim Turret Test", group = "Testers")
public class AutoAimTurretTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, false);
        AutoAimTurret autoAimTurret = new AutoAimTurret(hardwareMap, Alliance.BLUE, follower);

        Pose leftCornerPose = FieldMap.LEFT_CORNER_POSE;

        Pose rightCornerPose = FieldMap.RIGHT_CORNER_POSE;

        follower.setStartingPose(leftCornerPose);

        telemetry.addLine("AutoAim Turret Test Ready!");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Gamepad 1 Left Stick : Drive Forward/Backward & Strafe");
        telemetry.addLine("  Gamepad 1 Right Stick: Turn");
        telemetry.addLine("  Gamepad 1 X          : Reset Odometry to LEFT Corner");
        telemetry.addLine("  Gamepad 1 B          : Reset Odometry to RIGHT Corner");
        telemetry.addLine("  Gamepad 1 Y          : Reset Odometry to Field Center");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive();

        boolean xWasPressed = false;
        boolean bWasPressed = false;
        boolean yWasPressed = false;

        while (opModeIsActive()) {
            follower.update();

            // Drive control
            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            drivetrain.drive(driveY, driveX, turn);

            // Odometry Reset Controls
            if (gamepad1.x && !xWasPressed) {
                follower.setPose(leftCornerPose);
            }
            if (gamepad1.b && !bWasPressed) {
                follower.setPose(rightCornerPose);
            }
            if (gamepad1.y && !yWasPressed) {
                follower.setPose(new Pose(0, 0, AngleUnit.Pif));
            }

            xWasPressed = gamepad1.x;
            bWasPressed = gamepad1.b;
            yWasPressed = gamepad1.y;

            // Update auto-aiming turret solution
            autoAimTurret.update();

            // Telemetry output
            Pose currentPose = follower.getPose();
            telemetry.addData("Status", "Running");
            telemetry.addData("Robot Pose", "X: %.1f in, Y: %.1f in, Heading: %.1f°",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            autoAimTurret.addTelemetry(telemetry);
            telemetry.update();
        }

        drivetrain.stop();
    }
}
