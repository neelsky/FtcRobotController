package org.firstinspires.ftc.teamcode.raahi.autos.blue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;


@Autonomous(name = "Distance To Hive (Limelight)", group = "Auto")
public class blueGarden extends LinearOpMode {

    private DcMotorEx outtake;

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver odometry;

    public static double NEW_P = 7.0;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 15.0;

    static final double mmPerFoot   = 304.8;
    static final double mmErrorTolerance  = 25;
    static final double maxPower     = 0.6;
    static final double minPower     = 0.2;
    static final double brakeDistance   = 0.002;
    static final double heading    = 1.0;

    boolean isAtGoalVelocity = false;

    double range = 0.02;
    double minRange;
    double maxRange;

    private static final int scoringSideTag = 40;
    private static final int audienceSideTag = 44;
    private static final int aprilTagPipeline = 0;

    private Limelight3A limelight;

    int goalVelocity = 0;

    double distanceToTarget = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(aprilTagPipeline);


        waitForStart();
        limelight.start();

        initializeMotors();

        while (opModeIsActive()) {
            LLResultTypes.FiducialResult upTag = findUpTag();
            if(upTag != null){
                setGoalVelocity(upTag);
                runOuttakeMotor();
            }else{
                goalVelocity = 0;
                outtake.setVelocity(0);

            }
        }

        limelight.stop();
    }


    private LLResultTypes.FiducialResult findUpTag(){
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
        return upTag;
    }



    private void setGoalVelocity(LLResultTypes.FiducialResult upTag) {

        distanceToTarget = distanceToTagInches(upTag);
        telemetry.addData("distance", "%.1f in", distanceToTarget);

        if (distanceToTarget > 40 && distanceToTarget < 140) {
            goalVelocity = (int) (941.2069 + 0.4127235 * Math.pow(distanceToTarget, 1.4620166) + 147);
            telemetry.addData("goalVelocity", goalVelocity);
        }
    }

    private void runOuttakeMotor() {
        double currentVelocity = outtake.getVelocity();
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

    private void initializeMotors() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");


        // Assign the FIELD (no "Limelight3A" in front), so the other methods can use it
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();



        // setVelocity needs the encoder mode
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        PIDFCoefficients pidfValues = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Flywheel PIDF",
                "P=%.6f I=%.6f D=%.6f F=%.6f",
                pidfValues.p, pidfValues.i, pidfValues.d, pidfValues.f);
        telemetry.update();

        leftFront  = hardwareMap.get(DcMotor.class, "frontleft");
        rightFront = hardwareMap.get(DcMotor.class, "frontright");
        leftBack   = hardwareMap.get(DcMotor.class, "backleft");
        rightBack  = hardwareMap.get(DcMotor.class, "backright");
        odometry   = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
    }
}