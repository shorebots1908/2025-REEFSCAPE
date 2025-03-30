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
  public final boolean encoderInvert;
  public final boolean motorInvert;
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
      boolean encoderInvert,
      boolean motorInvert,
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
    this.motorInvert = motorInvert;
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
      boolean motorInvert,
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
    this.motorInvert = motorInvert;
    this.encoderInvert = encoderInvert;
    this.startPosition = startPosition;
  }
}
