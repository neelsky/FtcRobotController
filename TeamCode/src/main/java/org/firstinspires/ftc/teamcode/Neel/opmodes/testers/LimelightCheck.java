package org.firstinspires.ftc.teamcode.Neel.opmodes.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.aim.LimelightAim;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.Turret;

@TeleOp(name = "Limelight Check", group = "Glitch")
public class LimelightCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        LimelightAim camera = new LimelightAim(hardwareMap, Alliance.BLUE);
        Servo turret = hardwareMap.get(Servo.class, "turret");
        Alliance alliance = Alliance.BLUE;

        telemetry.addLine("Turret goes to straight back (its zero). Nothing else moves.");
        telemetry.addLine("Gamepad 1: X = blue tags, B = red tags.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            turret.setPosition(Turret.TURRET_ZERO_POS);
            if (gamepad1.x) {
                alliance = Alliance.BLUE;
                camera.setAlliance(alliance);
            }
            if (gamepad1.b) {
                alliance = Alliance.RED;
                camera.setAlliance(alliance);
            }
            camera.update(0, 0, Turret.PIVOT_FORWARD_IN, Turret.PIVOT_LEFT_IN);

            telemetry.addData("alliance", alliance);
            if (camera.sees()) {
                telemetry.addData("cell", "%d from %d tag(s): %s", camera.getCell(), camera.getTagCount(), camera.getIds());
                telemetry.addData("cell middle from robot center", "%.1f in forward, %.1f in left, %.1f in up",
                        camera.getForwardIn(), camera.getLeftIn(), camera.getUpIn());
                telemetry.addData("picture age", "%.0f ms", camera.getAgeMs());
            } else {
                telemetry.addLine("no hive tags for this alliance");
            }
            telemetry.addLine("");
            telemetry.addLine("Turret faces BACK, so the tags should read NEGATIVE forward.");
            telemetry.addLine("Tags to the robot's left should read POSITIVE left. If it's the other way, flip LimelightAim.ROBOT_SPACE_Y_IS_RIGHT.");
            telemetry.addLine("If the distance is off, check the tag size in the Limelight pipeline (82.55 mm).");
            telemetry.update();
        }
        camera.stop();
    }
}
