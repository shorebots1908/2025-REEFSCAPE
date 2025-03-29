package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public double yellow = 0.69;
    public double purple = 0.91;
    public double green = 0.77;
    public double red = 0.61;
    public double blue = 0.87;
    public double teamColor;
    public String color = "";
  }

  public default void periodic() {}

  public default void updateInputs(LEDIOInputs inputs) {}

  public default String name() {
    return "LED";
  }

  public default void setLEDColor(double pwmColorCode) {}

  public default void currentLEDColor() {}

  public default void yellow() {}

  public default void green() {}

  public default void blue() {}

  public default void red() {}

  public default void purple() {}

  public default void teamColor() {}

  public default void updateTeamColor() {}
}
