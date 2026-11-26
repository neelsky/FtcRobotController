package org.firstinspires.ftc.teamcode.Neel.subsystems.turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Neel.subsystems.LimelightVision;
import org.firstinspires.ftc.teamcode.Neel.subsystems.map.FieldMapMetric;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * AutoTurret controls automatic aiming of a turret using odometry (PedroPathing)
 * and vision (Limelight AprilTags).
 * It embeds a {@link BasicTurret} to perform physical servo control and handles
 * position tracking, target selection, distance calculations, and vision fusion.
 */
public class AutoAimTurret {

    // Geometry Offsets
    public static double TURRET_FORWARD_OF_CENTER_IN = -9.74;
    public static double TURRET_LEFT_OF_CENTER_IN = 0.0;
    public static double TURRET_ZERO_FROM_FRONT_DEG = 180.0; // Angle facing backwards relative to robot front

    // Aiming Thresholds
    public static double AIM_TOLERANCE_DEG = 3.0;



    // Subsystems
    private final BasicTurret basicTurret;
    private final Follower follower;
    private final LimelightVision vision;
    private Alliance alliance;

    private double calculatedTargetAngleDeg = 0.0;
    private double distanceToTargetInches = 0.0;

    private boolean aimKnown = false;
    private boolean visionActive;
    private String aimSource = "None";

    /**
     * Constructs AutoTurret using a HardwareMap and specified Alliance.
     */
    public AutoAimTurret(HardwareMap hardwareMap, Alliance alliance, Follower follower) {
        this.basicTurret = new BasicTurret(hardwareMap);
        this.alliance = alliance;

        this.follower = follower;

        this.vision = new LimelightVision(hardwareMap, alliance);
    }

    /**
     * Main update loop. Updates follower pose (if present), calculates
     * aiming angle, and commands the underlying BasicTurret.
     */
    public void update() {
        Pose robotPose = null;
        if (follower != null) {
          follower.updatePose();
          robotPose = follower.getPose();
        }
        update(robotPose);
    }

    /**
     * Main update loop given a specific robot pose.
     *
     * @param robotPose current robot pose (x, y, heading)
     */
    public void update(Pose robotPose) {
        if (follower != null && robotPose == null) {
            follower.updatePose();
            robotPose = follower.getPose();
        }

        if (robotPose == null || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())) {
            aimKnown = false;
            aimSource = "No Odometry";
            return;
        }

        // Query target position dynamically using FieldMapMetric (symmetric across colors)
       Position targetPos = FieldMapMetric.getTargetFromPos(
            DistanceUnit.METER.fromInches(robotPose.getX()),
            DistanceUnit.METER.fromInches(robotPose.getY()));

        // If target is null, the robot is under the hive: do not make any adjustment to the turret
        if (targetPos == null) {
            aimSource = "Under Hive (No Adjustment)";
            return;
        }

        OdometryAim odometryAim = aimWithOdometry(robotPose, targetPos);
        vision.update();

        // Fuse vision angle if valid, otherwise rely on odometry
        if (vision.isTagVisible()) {
            // Vision provides relative offset to target
            calculatedTargetAngleDeg = basicTurret.getCurrentAngleDeg() + vision.getAngle();
            distanceToTargetInches = vision.getDistance();
            aimSource = "Limelight Vision (" + vision.getTagsUsed() + " tags)";
        } else {
            calculatedTargetAngleDeg = odometryAim.angleDegree;
            distanceToTargetInches = odometryAim.distance;
            aimSource = "Odometry";
        }

        aimKnown = true;

        basicTurret.setAngle(odometryAim.angleDegree);
        // shooter.setDistance(distanceToTargetInches);
    }

    public OdometryAim aimWithOdometry(Pose robotPose, Position targetPos) {
      OdometryAim aimData = new OdometryAim();

      // 1. Calculate turret world coordinates based on robot offsets
      double robotHeadingRad = robotPose.getHeading();
      double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

      double turretWorldX = robotPose.getX()
          + TURRET_FORWARD_OF_CENTER_IN * Math.cos(robotHeadingRad)
          - TURRET_LEFT_OF_CENTER_IN * Math.sin(robotHeadingRad);
      double turretWorldY = robotPose.getY()
          + TURRET_FORWARD_OF_CENTER_IN * Math.sin(robotHeadingRad)
          + TURRET_LEFT_OF_CENTER_IN * Math.cos(robotHeadingRad);

      // 2. Compute distance and bearing from turret to target
      double dx = targetPos.x - turretWorldX;
      double dy = targetPos.y - turretWorldY;
      aimData.distance = Math.hypot(dx, dy);

      double targetBearingDeg = Math.toDegrees(Math.atan2(dy, dx));

      // 3. Compute target turret angle relative to robot heading
      aimData.angleDegree = normalizeDegrees(targetBearingDeg - robotHeadingDeg - TURRET_ZERO_FROM_FRONT_DEG);

      return aimData;
    }

    /**
     * Checks if the turret is aligned with the target within tolerance.
     */
    public boolean isAimed() {
        if (!aimKnown || !canReach()) {
            return false;
        }
        double error = Math.abs(normalizeDegrees(basicTurret.getCurrentAngleDeg() - calculatedTargetAngleDeg));
        return error <= AIM_TOLERANCE_DEG;
    }

    /**
     * Checks if the target angle is within the turret's reach.
     */
    public boolean canReach() {
        return basicTurret.canReach(calculatedTargetAngleDeg);
    }

    /**
     * Returns true if aiming solution is valid.
     */
    public boolean isAimKnown() {
        return aimKnown;
    }

    /**
     * Returns distance to current target in inches.
     */
    public double getDistanceToTargetInches() {
        return distanceToTargetInches;
    }

    /**
     * Returns calculated target angle in degrees.
     */
    public double getCalculatedTargetAngleDeg() {
        return calculatedTargetAngleDeg;
    }

    /**
     * Returns internal BasicTurret subsystem.
     */
    public BasicTurret getBasicTurret() {
        return basicTurret;
    }

    /**
     * Returns internal Vision subsystem.
     */
    public LimelightVision getVision() {
        return vision;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        if (vision != null) {
            vision.setAlliance(alliance);
        }
    }

    public boolean isVisionActive() {
        return visionActive;
    }

    public void setVisionActive(boolean visionActive) {
        this.visionActive = visionActive;
    }

    /**
     * Normalizes an angle in degrees into the range [-180, 180].
     */
    public static double normalizeDegrees(double angleDeg) {
        return AngleUnit.normalizeDegrees(angleDeg);
    }

    /**
     * Displays subsystem diagnostic telemetry.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret Aim Source", aimSource);
        telemetry.addData("Turret Target Angle", "%.1f°", calculatedTargetAngleDeg);
        telemetry.addData("Turret Current Angle", "%.1f°", basicTurret.getCurrentAngleDeg());
        telemetry.addData("Distance to Target", "%.1f in", distanceToTargetInches);
        telemetry.addData("Turret Aimed", isAimed() ? "READY" : "TURNING");
    }
}
