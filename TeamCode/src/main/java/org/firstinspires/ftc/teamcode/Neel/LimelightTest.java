package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;


@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : tags) {
                    // Tag position relative to the camera
                    Position pos = tag.getTargetPoseCameraSpace().getPosition();
                    double x = pos.unit.toInches(pos.x);
                    double y = pos.unit.toInches(pos.y);
                    double z = pos.unit.toInches(pos.z);

                    // Straight-line distance from the camera to the tag
                    double distance = Math.sqrt(x * x + y * y + z * z);

                    telemetry.addData("Tag " + tag.getFiducialId(),
                            "%.1f in away", distance);
                }
            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}