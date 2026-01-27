package frc.robot.subsystems.hood;

public class HoodConfig {
  public final String name;
  public final int pwmChannel;
  public final boolean invert;

  public HoodConfig(String name, int pwmChannel, boolean invert) {
    this.name = name;
    this.pwmChannel = pwmChannel;
    this.invert = invert;
  }

  public HoodConfig(String name, int pwmChannel) {
    this(name, pwmChannel, false);
  }
}
