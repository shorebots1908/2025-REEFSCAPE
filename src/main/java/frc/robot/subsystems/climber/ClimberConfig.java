package frc.robot.subsystems.climber;

public class ClimberConfig {
  public final int leftMotorCanId;
  public final int rightMotorCanId;
  public final double pGain;
  public final double iGain;
  public final double dGain;
  public final double encoderLowerLimit;
  public final double encoderUpperLimit;

  public ClimberConfig(
      int leftMotorId,
      int rightMotorId,
      double pGain,
      double iGain,
      double dGain,
      double encoderLowerLimit,
      double encoderUpperLimit) {
    this.leftMotorCanId = leftMotorId;
    this.rightMotorCanId = rightMotorId;
    this.pGain = pGain;
    this.iGain = iGain;
    this.dGain = dGain;
    this.encoderLowerLimit = encoderLowerLimit;
    this.encoderUpperLimit = encoderUpperLimit;
  }
}
