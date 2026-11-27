package org.firstinspires.ftc.teamcode.Neel.opmodes.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Neel.subsystems.StatusLight;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.Intake;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.Shooter;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.Transfer;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.Turret;
import org.firstinspires.ftc.teamcode.Neel.subsystems.robot.turret.aim.OdometryAim;

@TeleOp(name = "Shot Table Tuner", group = "Glitch")
public class ShotTableTuner extends LinearOpMode {
    public static int SMALL_STEP = 10;
    public static int BIG_STEP = 50;
    public static double TRIM_STEP_IN = 0.5;

    private boolean upWas, downWas;
    private int stickYWas, stickXWas;
    private boolean intakePulling = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);
        Turret aim = new Turret(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, false);
        Shooter shooter = new Shooter(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        StatusLight light = new StatusLight(hardwareMap);

        while (opModeInInit()) {
            aim.initLoop(gamepad1.x, gamepad1.b, gamepad1.back);
            light.showAimed(false);
            telemetry.addLine("SHOT TABLE TUNER (same buttons as Odometry Turret Shooting)");
            telemetry.addLine("Drive to a spot, hold gamepad 2 RIGHT TRIGGER to shoot, then fix that distance's speed with gamepad 2 dpad up/down.");
            aim.addInitTelemetry(telemetry);
            telemetry.update();
        }

        aim.start();
        while (opModeIsActive()) {
            boolean shooting = gamepad2.right_trigger > 0.1;
            aim.update(gamepad1.back, gamepad1.right_stick_x, shooting, gamepad2.y);

            double distance = aim.isAimKnown() ? aim.getDistanceToCell() : -1;
            int point = distance > 0 ? Shooter.tablePointFor(distance) : -1;
            int step = gamepad2.right_bumper ? BIG_STEP : SMALL_STEP;
            if (point >= 0 && gamepad2.dpad_up && !upWas) {
                Shooter.SPEED_TABLE[point] = Math.min(Shooter.MAX_VELOCITY, Shooter.SPEED_TABLE[point] + step);
            }
            if (point >= 0 && gamepad2.dpad_down && !downWas) {
                Shooter.SPEED_TABLE[point] = Math.max(0, Shooter.SPEED_TABLE[point] - step);
            }
            upWas = gamepad2.dpad_up;
            downWas = gamepad2.dpad_down;

            int stickY = gamepad2.left_stick_y < -0.7 ? 1 : gamepad2.left_stick_y > 0.7 ? -1 : 0;
            int stickX = gamepad2.left_stick_x > 0.7 ? 1 : gamepad2.left_stick_x < -0.7 ? -1 : 0;
            if (stickY != 0 && stickYWas == 0) {
                OdometryAim.CELL_CENTER_OUT_TRIM_IN += stickY * TRIM_STEP_IN;
                aim.aimNow();
            }
            if (stickX != 0 && stickXWas == 0) {
                OdometryAim.CELL_CENTER_SIDE_TRIM_IN += stickX * TRIM_STEP_IN;
                aim.aimNow();
            }
            stickYWas = stickY;
            stickXWas = stickX;

            if (!shooting) {
                shooter.setGoal(0);
            } else if (point >= 0) {
                shooter.setGoal(Shooter.velocityForDistance(distance));
            } else {
                shooter.useDefaultIfOff();
            }
            shooter.update();
            if (gamepad2.left_trigger > 0.1) {
                transfer.forceFeed();
            } else {
                transfer.runTriggerFeed(shooter, shooting, aim.isAimed());
            }

            if (gamepad1.dpad_up) {
                intake.setIntake(Intake.IN_POWER);
                intakePulling = false;
            }
            if (gamepad1.dpad_down) {
                intake.setIntake(Intake.OUT_POWER);
                intakePulling = true;
            }
            if (gamepad1.dpad_left) {
                intake.setIntake(0);
                intakePulling = false;
            }
            if (gamepad2.dpad_right) {
                intake.flowerRight();
            }
            if (gamepad2.dpad_left) {
                intake.flowerLeft();
            }
            if (gamepad1.right_bumper) {
                shooter.stop();
                transfer.stop();
                intake.setIntake(0);
                intakePulling = false;
            }
            if (intakePulling && !transfer.isFeeding() && gamepad2.left_trigger <= 0.1) {
                transfer.runBackward();
            }
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            light.showAimed(aim.isAimed());

            if (point >= 0) {
                telemetry.addData("distance", "%.1f in   ->  tuning the %.0f in point", distance, Shooter.tableDistance(point));
                telemetry.addData(String.format("%.0f in speed", Shooter.tableDistance(point)), "%d   (gamepad 2 dpad up/down = %d, hold gamepad 2 right bumper = %d)",
                        Shooter.SPEED_TABLE[point], SMALL_STEP, BIG_STEP);
                telemetry.addData("speed used right here", "%d (between the two nearest points)", Shooter.velocityForDistance(distance));
            } else {
                telemetry.addLine("Distance unknown: press gamepad 2 Y to look for the tags, or gamepad 1 BACK on the start spot.");
            }
            telemetry.addData("ready", aim.isAimed() ? (shooter.atSpeed() ? "YES" : "aimed, flywheel spinning up") : "NO - turret not aimed");
            telemetry.addLine("Gamepad 2: right trigger = shoot, left trigger = force feed, Y = find tags, dpad right/left = flower");
            telemetry.addLine("Gamepad 1: dpad up/down/left = intake, right bumper = stop all");
            telemetry.addData("CELL MIDDLE TRIM", "out %.1f in (gamepad 2 left stick up/down), side %.1f in (gamepad 2 left stick left/right)",
                    OdometryAim.CELL_CENTER_OUT_TRIM_IN, OdometryAim.CELL_CENTER_SIDE_TRIM_IN);
            telemetry.addLine("Flick the stick until the turret points at the middle from two different spots, then copy both into OdometryAim.java.");
            telemetry.addLine("");
            telemetry.addLine("Copy this into Shooter.java (SPEED_TABLE, 25 in to 125 in):");
            addTable(telemetry);
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }
        shooter.stop();
        transfer.stop();
        intake.stop();
        aim.stop();
    }

    private void addTable(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        int[] table = Shooter.SPEED_TABLE;
        for (int start = 0; start < table.length; start += 7) {
            StringBuilder row = new StringBuilder();
            for (int i = start; i < Math.min(start + 7, table.length); i++) {
                row.append(table[i]);
                row.append(i == table.length - 1 ? "" : ", ");
            }
            telemetry.addLine("        " + row);
        }
    }
}
