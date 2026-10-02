package org.firstinspires.ftc.teamcode.raahi.autos.blue;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;


@Autonomous(name = "Distance To Hive (Limelight)", group = "Auto")
public class blueOtherSide extends LinearOpMode {

    private static final int scoringSideTag = 40;
    private static final int audienceSideTag = 44;
    private static final int aprilTagPipeline = 0;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(aprilTagPipeline);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            LLResultTypes.FiducialResult upTag = null;
            double upTagHeight = Double.NEGATIVE_INFINITY;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : tags) {
                    int id = tag.getFiducialId();
                    if (id != scoringSideTag && id != audienceSideTag) {
                        continue; // ignore all other tags
                    }

                    // Keep whichever hive tag is highest (the side that's up).
                    double height = tagHeightInches(tag);
                    if (upTag == null || height > upTagHeight) {
                        upTag = tag;
                        upTagHeight = height;
                    }
                }
            }

            if (upTag == null) {
                telemetry.addData("Distance to hive", "no tag in view");
            } else {
                telemetry.addData("Distance to hive", "%.1f in", distanceToTagInches(upTag));
            }
            telemetry.update();
        }

        limelight.stop();
    }

    /** Height (Z) of the tag in robot space, in inches. */
    private double tagHeightInches(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
        return p.z;
    }

    /** Straight-line distance from the camera to the tag, in inches. */
    private double distanceToTagInches(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
        return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }
}