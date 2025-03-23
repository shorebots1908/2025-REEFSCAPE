package frc.robot.subsystems.wrist;

import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = false;
    public double positionRevs = 0.0;
    public int sensorValue = 0;
  }

  public default void periodic() {}

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setTargetPosition(BasePosition position) {}

  public default boolean atTargetPosition() {
    return false;
  }

  public default String name() {
    return "Wrist";
  }

  public default void setWristOpenLoop(double output) {}
}
