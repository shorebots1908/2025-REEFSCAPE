package frc.robot.subsystems.wrist;

import frc.robot.subsystems.BasePosition;

public class WristConfig {
  public final String name;
  public final int wristMotorId;
  public final double pGain;
  public final double iGain;
  public final double dGain;
  public final double rampRate;
  public final double encoderLowerLimit;
  public final double encoderUpperLimit;
  public final boolean softLimitEnabled;
  public final boolean wristInvert;
  public final boolean encoderInvert;
  public final BasePosition startPosition;

  public WristConfig(
      String name,
      int wristMotorId,
      double p,
      double i,
      double d,
      double rampRate,
      double lowerLimit,
      double upperLimit,
      boolean softLimitEnabled,
      boolean wristInvert,
      boolean encoderInvert,
      BasePosition startPosition) {
    this.name = name;
    this.wristMotorId = wristMotorId;
    this.pGain = p;
    this.iGain = i;
    this.dGain = d;
    this.rampRate = rampRate;
    this.encoderLowerLimit = lowerLimit;
    this.encoderUpperLimit = upperLimit;
    this.softLimitEnabled = softLimitEnabled;
    this.wristInvert = wristInvert;
    this.encoderInvert = encoderInvert;
    this.startPosition = startPosition;
  }
}
