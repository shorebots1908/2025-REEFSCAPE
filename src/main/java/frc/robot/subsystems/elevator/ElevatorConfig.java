package frc.robot.subsystems.elevator;

public class ElevatorConfig {

  // Required
  public final int leftMotorCanId;
  public final int rightMotorCanId;
  public final int lowerLimitId;
  public final int upperLimitId;
  public final double lowerEncoderValue;
  public final double upperEncoderValue;

  public ElevatorConfig(
      int leftMotorId,
      int rightMotorId,
      int lowerLimitId,
      int upperLimitId,
      double lowerEncoderValue,
      double upperEncoderValue) {
    this.leftMotorCanId = leftMotorId;
    this.rightMotorCanId = rightMotorId;
    this.lowerLimitId = lowerLimitId;
    this.upperLimitId = upperLimitId;
    this.lowerEncoderValue = lowerEncoderValue;
    this.upperEncoderValue = upperEncoderValue;
  }
}
