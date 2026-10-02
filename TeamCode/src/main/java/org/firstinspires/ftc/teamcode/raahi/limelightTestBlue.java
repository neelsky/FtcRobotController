package org.firstinspires.ftc.teamcode.raahi;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;

@TeleOp(name = "limelightTest", group = "limelightTest")
public class limelightTestBlue extends LinearOpMode {

    // Specific tag marker
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int APRILTAG_PIPELINE = 0;

    // Change to whatever is needed
    private int targetTagID = 21;


    // Change to whichever buttons are needed, will need to create two, one for each team
    private boolean previousA = false;
    private boolean previousB = false;

    private Limelight3A limelight;

    @Override
    public void runOpMode(){
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        limelight.start();
        telemetry.addLine("");
        telemetry.addLine("Limelight initialized");
        telemetry.update();

        boolean targetFound = false;

        while (opModeIsActive()){

            // Can change to whatever values are needed
            // Having operator control
            if (gamepad2.a && !previousA) {
                targetTagID = 30;
            }

            if (gamepad2.b && !previousB) {
                targetTagID = 31;
            }

            previousA = gamepad2.a;
            previousB = gamepad2.b;

            LLResult result = limelight.getLatestResult();



            if(result != null && result.isValid()){
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();


                for (LLResultTypes.FiducialResult fiducial : fiducials) {

                    int tagID = fiducial.getFiducialId();

                    if (tagID != targetTagID) {
                        continue;
                    }

                    targetFound = true;
                    double targetX = fiducial.getTargetXDegrees();
                    double targetY = fiducial.getTargetYDegrees();

                    telemetry.addData("Target X", "%.2f degrees", targetX);

                    telemetry.addData("Target Y", "%.2f degrees", targetY);

                    break;

                }
            }




        }

        if (targetFound = false) {

            telemetry.addLine("TARGET NOT VISIBLE");

        }

        telemetry.update();
        sleep(20);

    }




}
