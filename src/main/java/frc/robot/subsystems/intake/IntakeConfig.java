package frc.robot.subsystems.intake;

import frc.robot.subsystems.BasePosition;
import java.util.Optional;

public class IntakeConfig {
  public final String name;
  public final int leftMotorId;
  public final Optional<Integer> rightMotorId;
  public final Optional<Integer> sensorId;
  public final Optional<Integer> sensorThreshold;
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
    this.rightMotorId = Optional.of(rightMotorId);
    this.sensorId = Optional.of(sensorId);
    this.sensorThreshold = Optional.of(sensorThreshold);
    this.pGain = p;
    this.iGain = i;
    this.dGain = d;
    this.rampRate = rampRate;
    this.encoderLowerLimit = lowerLimit;
    this.encoderUpperLimit = upperLimit;
    this.encoderInvert = encoderInvert;
    this.startPosition = startPosition;
  }

  public IntakeConfig(
      String name,
      int leftMotorId,
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
    this.rightMotorId = Optional.empty();
    this.sensorId = Optional.empty();
    this.sensorThreshold = Optional.empty();
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
