package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double positionRevs = 0.0;
    public int sensorValue = 0;
  }

  public default void periodic() {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default String name() {
    return "Intake";
  }

  public default void setFeedOpenLoop(double output) {}

  public default void feedStop() {}

  public default boolean isHolding() {
    return false;
  }

  public default boolean isEmpty() {
    return false;
  }
}
