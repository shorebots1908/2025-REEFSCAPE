package frc.robot.subsystems.spindexer;

public class SpindexerConfig {
  public final String name;
  public final int motorId;
  public final boolean motorInvert;
  public final int currentLimit;

  public SpindexerConfig(String name, int motorId, boolean motorInvert, int currentLimit) {
    this.name = name;
    this.motorId = motorId;
    this.motorInvert = motorInvert;
    this.currentLimit = currentLimit;
  }

  public SpindexerConfig(String name, int motorId, boolean motorInvert) {
    this(name, motorId, motorInvert, 40);
  }
}
