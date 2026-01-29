package frc.robot.subsystems.hood;

public interface HoodIO {
  public static class HoodIOInputs {
    public double position = 0.0;
  }

  public default void periodic() {}

  public default void updateInputs(HoodIOInputs inputs) {}

  public default String name() {
    return "Hood";
  }

  /** Set position from 0.0 (retracted) to 1.0 (extended) */
  public default void setPosition(double position) {}
}
