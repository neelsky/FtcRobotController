package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightVision {

  public static int LIMELIGHT_PIPELINE = 0;
  public static double MAX_CAMERA_FRAME_AGE_MS = 300.0;


  private final Limelight3A limelight;
  private Alliance alliance;


  private boolean tagVisible = false;
  private int tagsUsed = 0;
  private double distanceToTarget = -1;
  private double angleToTarget = 0;
  private boolean freshTag = false;
  private double frameAgeMs = 1e9;


  public LimelightVision(HardwareMap hardwareMap, Alliance alliance) {
    this.alliance = alliance;
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
    limelight.start();
  }

  public void update() {
    // compute the angle and distance.
  }

  public boolean isTagVisible() {
    return tagVisible;
  }

  public double getDistance() {
    return distanceToTarget;
  }

  public double getAngle() {
    return angleToTarget;
  }

  public boolean hasFreshTag() {
    return freshTag;
  }

  public int getTagsUsed() {
    return tagsUsed;
  }

  public Alliance getAlliance() {
    return alliance;
  }

  public double getFrameAgeMs() {
    return frameAgeMs;
  }

  public void setAlliance(Alliance newAlliance) {
    if (newAlliance == alliance) {
      return;
    }
    alliance = newAlliance;
    resetTracking();
  }

  public void resetTracking() {
    tagVisible = false;
    tagsUsed = 0;
    distanceToTarget = -1;
  }
}
