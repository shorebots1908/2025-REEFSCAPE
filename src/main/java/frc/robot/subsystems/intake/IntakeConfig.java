package frc.robot.subsystems.intake;

import frc.robot.subsystems.BasePosition;

public class IntakeConfig {
  public final String name;
  public final int leftMotorId;
  public final int rightMotorId;
  public final int wristMotorId;
  public final int sensorId;
  public final int sensorThreshold;
  public final double pGain;
  public final double iGain;
  public final double dGain;
  public final double rampRate;
  public final double encoderLowerLimit;
  public final double encoderUpperLimit;
  public final boolean encoderInvert;
  public final BasePosition startPosition;

  public IntakeConfig(
      String name,
      int leftMotorId,
      int rightMotorId,
      int wristMotorId,
      int sensorId,
      int sensorThreshold,
      double p,
      double i,
      double d,
      double rampRate,
      double lowerLimit,
      double upperLimit,
      boolean encoderInvert,
      BasePosition startPosition) {
    this.name = name;
    this.leftMotorId = leftMotorId;
    this.rightMotorId = rightMotorId;
    this.wristMotorId = wristMotorId;
    this.sensorId = sensorId;
    this.sensorThreshold = sensorThreshold;
    this.pGain = p;
    this.iGain = i;
    this.dGain = d;
    this.rampRate = rampRate;
    this.encoderLowerLimit = lowerLimit;
    this.encoderUpperLimit = upperLimit;
    this.encoderInvert = encoderInvert;
    this.startPosition = startPosition;
  }
}
