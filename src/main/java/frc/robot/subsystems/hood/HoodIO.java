package frc.robot.subsystems.hood;

public interface HoodIO {
  public static class HoodIOInputs {
    public double appliedOutput = 0.0;
  }

  public default void periodic() {}

  public default void updateInputs(HoodIOInputs inputs) {}

  public default String name() {
    return "Hood";
  }

  public default void setOpenLoop(double output) {}

  public default void stop() {}
}
