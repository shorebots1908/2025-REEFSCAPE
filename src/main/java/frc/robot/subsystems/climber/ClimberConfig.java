package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberConfig {
  public final int leftMotorCanId;
  public final int rightMotorCanId;
  public final double pGain;
  public final double iGain;
  public final double dGain;
  public final Rotation2d encoderLowerLimit;
  public final Rotation2d encoderUpperLimit;

  public ClimberConfig(
      int leftMotorId,
      int rightMotorId,
      double pGain,
      double iGain,
      double dGain,
      Rotation2d encoderLowerLimit,
      Rotation2d encoderUpperLimit) {
    this.leftMotorCanId = leftMotorId;
    this.rightMotorCanId = rightMotorId;
    this.pGain = pGain;
    this.iGain = iGain;
    this.dGain = dGain;
    this.encoderLowerLimit = encoderLowerLimit;
    this.encoderUpperLimit = encoderUpperLimit;
  }
}
