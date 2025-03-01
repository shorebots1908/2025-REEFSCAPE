package frc.robot.hybrid;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * An encoding of all the dynamic control signals for the robot.
 *
 * <p>Dynamic controls are ones with a range (i.e. 0.0 to 1.0) rather than a binary button-like or
 * toggle-like control.
 */
public class ControlVector {
  private double swerveFieldX;
  private double swerveFieldY;
  private double swerveRobotX;
  private double swerveRobotY;
  private double swerveRotation;
  private double elevatorPosition;
  private double coralPosition;
  private double coralSpeed;
  private double algaePosition;
  private double algaeSpeed;
  private double climberPosition;

  /** Create a new ControlVector whose values are all zero */
  public ControlVector() {
    this(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  /** Create a new ControlVector with the given values */
  public ControlVector(
      double swerveFieldX,
      double swerveFieldY,
      double swerveRobotX,
      double swerveRobotY,
      double swerveRotation,
      double elevatorPosition,
      double coralPosition,
      double coralSpeed,
      double algaePosition,
      double algaeSpeed,
      double climberPosition) {
    this.swerveFieldX = swerveFieldX;
    this.swerveFieldY = swerveFieldY;
    this.swerveRobotX = swerveRobotX;
    this.swerveRobotY = swerveRobotY;
    this.swerveRotation = swerveRotation;
    this.elevatorPosition = elevatorPosition;
    this.coralPosition = coralSpeed;
    this.algaePosition = this.algaeSpeed = algaeSpeed;
    this.climberPosition = climberPosition;
  }

  /** Create a new ControlVector that's a copy of the given ControlVector */
  public ControlVector(ControlVector toCopy) {
    this(
        toCopy.swerveFieldX,
        toCopy.swerveFieldY,
        toCopy.swerveRobotX,
        toCopy.swerveRobotY,
        toCopy.swerveRotation,
        toCopy.elevatorPosition,
        toCopy.coralPosition,
        toCopy.coralSpeed,
        toCopy.algaePosition,
        toCopy.algaeSpeed,
        toCopy.climberPosition);
  }

  /**
   * Create a new ControlVector using the given field-relative X, Y, and rotation All other control
   * values are set to 0.
   *
   * @param swerveFieldX field-relative X power
   * @param swerveFieldY field-relative Y power
   * @param swerveRotation rotation power
   */
  public static ControlVector fromFieldRelative(
      double swerveFieldX, double swerveFieldY, double swerveRotation) {
    return new ControlVector(
        swerveFieldX,
        swerveFieldY,
        0.0, // robotX
        0.0, // robotY
        swerveRotation,
        0.0, // elevatorPosition
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);
  }

  /**
   * Create a new ControlVector using the given robot-relative X, Y, and rotation All other control
   * values are set to 0.
   *
   * @param swerveRobotX robot-relative X power
   * @param swerveRobotY robot-relative Y power
   * @param swerveRotation rotation power
   */
  public static ControlVector fromRobotRelative(
      double swerveRobotX, double swerveRobotY, double swerveRotation) {
    return new ControlVector(
        0.0,
        0.0,
        swerveRobotX,
        swerveRobotY,
        swerveRotation,
        0.0, // elevatorTarget
        0.0, // coralPosition
        0.0, // coralSpeed
        0.0, // algaePosition
        0.0, // algaeSpeed
        0.0 // climberPosition
        );
  }

  /**
   * @return the control value for the Swerve drive X translation (field space)
   */
  public double swerveFieldX() {
    return this.swerveFieldX;
  }

  /**
   * @return the control value for the Swerve drive Y translation (field space)
   */
  public double swerveFieldY() {
    return this.swerveFieldY;
  }

  /**
   * @return the control value for the Swerve drive X translation (robot space)
   */
  public double swerveRobotX() {
    return this.swerveRobotX;
  }

  /**
   * @return the control value for the Swerve drive Y translation (robot space)
   */
  public double swerveRobotY() {
    return this.swerveRobotY;
  }

  /**
   * @return the control value for Swerve drive rotation (-1.0 to 1.0)
   */
  public double swerveRotation() {
    return this.swerveRotation;
  }

  public double elevatorPosition() {
    return this.elevatorPosition;
  }

  public double coralPosition() {
    return this.coralPosition;
  }

  public double coralSpeed() {
    return this.coralSpeed;
  }

  public double algaePosition() {
    return this.algaePosition;
  }

  public double algaeSpeed() {
    return this.algaeSpeed;
  }

  public double climberPosition() {
    return this.climberPosition;
  }

  /** Set the swerve field-X coordinate of this ControlVector */
  public ControlVector setSwerveFieldX(double fieldX) {
    this.swerveFieldX = fieldX;
    return this;
  }

  /** Set the swerve field-X coordinate of this ControlVector */
  public ControlVector setSwerveFieldY(double fieldY) {
    this.swerveFieldY = fieldY;
    return this;
  }

  /** Set the swerve robot-X coordinate of this ControlVector */
  public ControlVector setSwerveRobotX(double robotX) {
    this.swerveRobotX = robotX;
    return this;
  }

  /** Set the swerve robot-Y coordinate of this ControlVector */
  public ControlVector setSwerveRobotY(double robotY) {
    this.swerveRobotY = robotY;
    return this;
  }

  /** Set the swerve rotation coordinate of this ControlVector */
  public ControlVector setSwerveRotation(double rotation) {
    this.swerveRotation = rotation;
    return this;
  }

  /**
   * Add the values of this vector to the corresponding values of the other vector. This
   * ControlVector remains unmodified.
   *
   * @return a new ControlVector with the calculated values
   */
  public ControlVector plus(ControlVector other) {
    return new ControlVector(
        this.swerveFieldX + other.swerveFieldX,
        this.swerveFieldY + other.swerveFieldY,
        this.swerveRobotX + other.swerveRobotX,
        this.swerveRobotY + other.swerveRobotY,
        this.swerveRotation + other.swerveRotation,
        this.elevatorPosition + other.elevatorPosition,
        this.coralPosition + other.coralPosition,
        this.coralSpeed + other.coralSpeed,
        this.algaePosition + other.algaePosition,
        this.algaeSpeed + other.algaeSpeed,
        this.climberPosition + other.climberPosition);
  }

  /**
   * Multiply the values of this vector by a given scalar value This ControlVector remains
   * unmodified.
   *
   * @return a new ControlVector with the calculated values
   */
  public ControlVector times(ControlVector other) {
    return new ControlVector(
        this.swerveFieldX * other.swerveFieldX,
        this.swerveFieldY * other.swerveFieldY,
        this.swerveRobotX * other.swerveRobotX,
        this.swerveRobotY * other.swerveRobotY,
        this.swerveRotation * other.swerveRotation,
        this.elevatorPosition * other.elevatorPosition,
        this.coralPosition * other.coralPosition,
        this.coralSpeed * other.coralSpeed,
        this.algaePosition * other.algaePosition,
        this.algaeSpeed * other.algaeSpeed,
        this.climberPosition * other.climberPosition);
  }

  /**
   * Multiply the values of this vector by a given scalar value This ControlVector remains
   * unmodified.
   *
   * @return a new ControlVector with the calculated values
   */
  public ControlVector scale(double scalar) {
    return new ControlVector(
        this.swerveFieldX * scalar,
        this.swerveFieldY * scalar,
        this.swerveRobotX * scalar,
        this.swerveRobotY * scalar,
        this.swerveRotation * scalar,
        this.elevatorPosition * scalar,
        this.coralPosition * scalar,
        this.coralSpeed * scalar,
        this.algaePosition * scalar,
        this.algaeSpeed * scalar,
        this.climberPosition * scalar);
  }

  /**
   * Calculate a ChassisSpeeds from both the field-relative controls and robot-relative controls
   *
   * @param heading the current heading of the robot
   */
  public ChassisSpeeds calculateChassisSpeeds(Rotation2d heading) {
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            swerveFieldX(), swerveFieldY(), swerveRotation(), heading);

    ChassisSpeeds robotRelativeSpeeds =
        new ChassisSpeeds(swerveRobotX(), swerveRobotY(), swerveRotation());

    return new ChassisSpeeds(
        fieldRelativeSpeeds.vxMetersPerSecond + robotRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond + robotRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond + robotRelativeSpeeds.omegaRadiansPerSecond);
  }

  /**
   * At t=0, return mode1, at t=1 return mode2, return interpolation between
   *
   * @param t The interpolation value from 0 to 1
   * @return An interpolation of this and other with the given tValue
   */
  public ControlVector interpolate(ControlVector other, double t) {
    ControlVector t1Scaled = this.scale(1 - t);
    ControlVector t2Scaled = other.scale(t);

    return t1Scaled.plus(t2Scaled);
  }
}
