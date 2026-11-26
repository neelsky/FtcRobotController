package org.firstinspires.ftc.teamcode.Neel.subsystems.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * BasicTurret controls a turret driven by a single servo.
 * Characteristics:
 * - Servo position range is [0.0, 1.0], where 0.5 is fixed as zero degrees (0°).
 * - Rotation is measured in degrees (Clockwise is positive, Counterclockwise is negative).
 * - Total physical servo range in degrees (e.g. 300°) converts degrees to servo position.
 * - Angle limits in both directions are configurable.
 */
public class BasicTurret {
  public static final double ZERO_SERVO_POSITION = 0.5;
  public static final double DEFAULT_SERVO_RANGE_DEGREES = 305.0; // Total physical sweep angle across [0.0, 1.0]

  private final Servo servo;

  // Configuration settings
  private double servoRangeDegrees = DEFAULT_SERVO_RANGE_DEGREES;
  private boolean clockwiseIsPositive = true; // true if higher servo position moves Clockwise (positive angle)

  // Configurable angle limits in degrees (CCW negative, CW positive)
  private double minAngleDeg = -150.0; // Counterclockwise limit
  private double maxAngleDeg = 150.0;  // Clockwise limit

  // Servo physical output bounds
  private double minServoPosition = 0.0;
  private double maxServoPosition = 1.0;

  // Current state
  private double targetAngleDeg = 0.0;
  private double currentServoPosition = ZERO_SERVO_POSITION;

  /**
   * Construct a BasicTurret from a HardwareMap with default device name "turret".
   */
  public BasicTurret(HardwareMap hardwareMap) {
    this(hardwareMap, "turret");
  }

  /**
   * Construct a BasicTurret from a HardwareMap with a custom servo device name.
   */
  public BasicTurret(HardwareMap hardwareMap, String servoName) {
    Servo hwServo;
    try {
      hwServo = hardwareMap.get(Servo.class, servoName);
    } catch (Exception ignored) {
      hwServo = null;
    }
    this.servo = hwServo;
    init();
  }

  /**
   * Construct a BasicTurret using an existing Servo object.
   */
  public BasicTurret(Servo servo) {
    this.servo = servo;
    init();
  }

  /**
   * Moves the turret back to 0 degrees (zero position 0.5).
   */
  public void init() {
    setAngle(0.0);
    setAngle(getCurrentAngleDeg());
  }

  /**
   * Sets the target angle of the turret in degrees.
   * Clockwise angles are positive, Counterclockwise angles are negative.
   * Clamps the target angle to the configured limits before moving the servo.
   *
   * @param angleDeg target angle in degrees
   */
  public void setAngle(double angleDeg) {
    double clampedAngle = clampAngle(angleDeg);
    this.targetAngleDeg = clampedAngle;
    double pos = angleToPosition(clampedAngle);
    setPosition(pos);
  }

  /**
   * Sets the raw servo position within [0.0, 1.0].
   * Updates targetAngleDeg accordingly.
   *
   * @param position target position [0.0, 1.0]
   */
  private void setPosition(double position) {
    double clampedPos = Math.max(minServoPosition, Math.min(maxServoPosition, position));
    this.currentServoPosition = clampedPos;
    this.targetAngleDeg = positionToAngle(clampedPos);
    if (servo != null) {
      servo.setPosition(clampedPos);
    }
  }

  /**
   * Converts an angle in degrees to a servo position in [0.0, 1.0].
   * 0° maps to fixed zero position 0.5.
   * Clockwise (+deg) increases position if clockwiseIsHigher is true.
   */
  private double angleToPosition(double angleDeg) {
    double offsetFromZero = angleDeg / servoRangeDegrees;
    double pos = clockwiseIsPositive ? (ZERO_SERVO_POSITION + offsetFromZero) : (ZERO_SERVO_POSITION - offsetFromZero);
    return Math.max(minServoPosition, Math.min(maxServoPosition, pos));
  }

  /**
   * Converts a servo position in [0.0, 1.0] to an angle in degrees.
   */
  private double positionToAngle(double position) {
    double offsetFromZero = clockwiseIsPositive ? (position - ZERO_SERVO_POSITION) : (ZERO_SERVO_POSITION - position);
    return offsetFromZero * servoRangeDegrees;
  }

  /**
   * Checks whether a target angle in degrees is within configured limits.
   */
  public boolean canReach(double angleDeg) {
    return angleDeg >= minAngleDeg && angleDeg <= maxAngleDeg;
  }

  /**
   * Clamps an angle in degrees to stay within the configured limits.
   */
  public double clampAngle(double angleDeg) {
    return Math.max(minAngleDeg, Math.min(maxAngleDeg, angleDeg));
  }

  // --- Configuration Methods ---

  /**
   * Sets the angular limits in degrees.
   *
   * @param minLimitDeg minimum angle in degrees (CCW limit, e.g. -135.0)
   * @param maxLimitDeg maximum angle in degrees (CW limit, e.g. +135.0)
   */
  public void setLimits(double minLimitDeg, double maxLimitDeg) {
    this.minAngleDeg = Math.min(minLimitDeg, maxLimitDeg);
    this.maxAngleDeg = Math.max(minLimitDeg, maxLimitDeg);
  }

  public double getMinAngleDeg() {
    return minAngleDeg;
  }

  public double getMaxAngleDeg() {
    return maxAngleDeg;
  }

  public double getServoRangeDegrees() {
    return servoRangeDegrees;
  }

  public void setServoRangeDegrees(double servoRangeDegrees) {
    this.servoRangeDegrees = servoRangeDegrees;
  }

  public boolean isClockwiseIsPositive() {
    return clockwiseIsPositive;
  }

  public void setClockwiseIsPositive(boolean clockwiseIsPositive) {
    this.clockwiseIsPositive = clockwiseIsPositive;
  }

  public void setServoLimits(double minServoPosition, double maxServoPosition) {
    this.minServoPosition = Math.max(0.0, Math.min(1.0, minServoPosition));
    this.maxServoPosition = Math.max(0.0, Math.min(1.0, maxServoPosition));
  }

  // --- State Getters ---
  public double getTargetAngleDeg() {
    return targetAngleDeg;
  }

  public double getCurrentAngleDeg() {
    return positionToAngle(getServoPosition());
  }

  private double getServoPosition() {
    return currentServoPosition;
  }

  public Servo getServo() {
    return servo;
  }

  public boolean isHardwarePresent() {
    return servo != null;
  }
}
