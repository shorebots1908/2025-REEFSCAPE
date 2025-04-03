package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double positionRevs = 0.0;
    public boolean holdingSwitchPressed = false;
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

  public default void feedInWait() {}

  public default boolean timer() {
    return false;
  }

  public default void timerStart() {}
}
