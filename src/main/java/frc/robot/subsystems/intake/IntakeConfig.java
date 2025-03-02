package frc.robot.subsystems.intake;

import frc.robot.subsystems.BasePosition;

public class IntakeConfig {
  public final String name;
  public final int leftMotorId;
  public final int rightMotorId;
  public final int wristMotorId;
  public final double pGain;
  public final double iGain;
  public final double dGain;
  public final double encoderLowerLimit;
  public final double encoderUpperLimit;
  public final boolean encoderInvert;
  public final BasePosition startPosition;

  public IntakeConfig(
      String name,
      int leftMotorId,
      int rightMotorId,
      int wristMotorId,
      double p,
      double i,
      double d,
      double lowerLimit,
      double upperLimit,
      boolean encoderInvert,
      BasePosition startPosition){
    this.name = name;
    this.leftMotorId = leftMotorId;
    this.rightMotorId = rightMotorId;
    this.wristMotorId = wristMotorId;
    this.pGain = p;
    this.iGain = i;
    this.dGain = d;
    this.encoderLowerLimit = lowerLimit;
    this.encoderUpperLimit = upperLimit;
    this.encoderInvert = encoderInvert;
    this.startPosition = startPosition;
  }
}
