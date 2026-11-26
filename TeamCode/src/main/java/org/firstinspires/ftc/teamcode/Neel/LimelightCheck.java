package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.FieldMap;
import org.firstinspires.ftc.teamcode.Neel.subsystems.old.Vision;

@TeleOp(name = "Limelight Check", group = "Glitch")
public class LimelightCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Vision blue = new Vision(hardwareMap, Alliance.BLUE);
        Vision red = new Vision(hardwareMap, Alliance.RED);

        int pipeline = Vision.PIPELINE;
        boolean upWas = false, downWas = false;

        telemetry.addLine("Point the Limelight at the hive tags and press START.");
        telemetry.addLine("Nothing moves in this OpMode.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !upWas && pipeline < 9) {
                pipeline++;
                limelight.pipelineSwitch(pipeline);
            }
            if (gamepad1.dpad_down && !downWas && pipeline > 0) {
                pipeline--;
                limelight.pipelineSwitch(pipeline);
            }
            upWas = gamepad1.dpad_up;
            downWas = gamepad1.dpad_down;

            LLStatus status = limelight.getStatus();
            telemetry.addData("Limelight", "%s  fps %d  temp %.0f C", status.getName(), (int) status.getFps(), status.getTemp());
            telemetry.addData("pipeline", "asked for %d, running %d, type %s", pipeline, status.getPipelineIndex(), status.getPipelineType());
            telemetry.addLine("Dpad up/down = try another pipeline. The type must be the AprilTag (fiducial) one.");
            telemetry.addData("code uses pipeline", "%d (Vision.PIPELINE)%s", Vision.PIPELINE,
                    pipeline == Vision.PIPELINE ? "" : "  <- if this pipeline works better, set Vision.PIPELINE = " + pipeline);

            LLResult result = limelight.getLatestResult();
            if (result == null) {
                telemetry.addLine("NO RESULT: Limelight not sending data (check USB cable / config name \"limelight\")");
            } else if (!result.isValid()) {
                telemetry.addData("result", "not valid (no targets), %d ms old", result.getStaleness());
            } else {
                telemetry.addData("picture age", "%.0f ms",
                        result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency());
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    String owner = Alliance.BLUE.isHiveTag(id) ? "BLUE hive, cell " + FieldMap.clusterOf(Alliance.BLUE.toBlueTag(id))
                            : Alliance.RED.isHiveTag(id) ? "RED hive, cell " + FieldMap.clusterOf(Alliance.RED.toBlueTag(id))
                            : "NOT a hive tag the code knows";
                    telemetry.addData("tag " + id, "%s | family %s | tx %.1f° ty %.1f° area %.2f",
                            owner, tag.getFamily(), tag.getTargetXDegrees(), tag.getTargetYDegrees(), tag.getTargetArea());
                    Pose3D pose = tag.getTargetPoseCameraSpace();
                    if (pose == null) {
                        telemetry.addLine("    no 3D position: turn on Full 3D in the pipeline");
                    } else {
                        Position p = pose.getPosition();
                        telemetry.addData("    3D (camera)", "right %.1f  down %.1f  forward %.1f in (raw, before size fix)",
                                p.unit.toInches(p.x), p.unit.toInches(p.y), p.unit.toInches(p.z));
                    }
                }
            }

            blue.update(0, -1);
            red.update(0, -1);
            showVision("BLUE code sees", blue);
            showVision("RED code sees", red);

            telemetry.addLine("");
            telemetry.addData("expected tag height above camera",
                    "UP cell %.1f in, DOWN cell %.1f in (if 'rise' below is far off, camera height/tilt settings are wrong)",
                    FieldMap.UP_TAGS_HEIGHT_IN - Vision.CAMERA_HEIGHT_IN, FieldMap.DOWN_TAGS_HEIGHT_IN - Vision.CAMERA_HEIGHT_IN);
            telemetry.update();
        }
        blue.stop();
    }

    private void showVision(String label, Vision vision) {
        if (!vision.hasFreshTag()) {
            telemetry.addData(label, "nothing usable");
            return;
        }
        telemetry.addData(label, "cell %d via %d tag(s) | angle %.1f° | distance %.1f in | flat %.1f in | rise %.1f in%s",
                vision.getTargetCluster(), vision.getTagsUsed(), vision.getAngle(), vision.getDistance(),
                vision.getFlatDistance(), vision.getRise(),
                vision.hasMeasuredScale() ? String.format(" | with 2-tag size fix: flat %.1f rise %.1f",
                        vision.getScaledFlatDistance(), vision.getScaledRise()) : "");
    }
}
