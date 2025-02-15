package frc.robot.subsystems.elevator;

import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean connected = false;
    public double positionRad = 0;
    public double velocityRadPerSec = 0;
    public boolean atUpper = false;
    public boolean atLower = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setTargetPosition(BasePosition position) {}

  public default boolean atTargetPosition() {
    return false;
  }

  public default void setElevatorOpenLoop(double output) {}

  public default void stop() {}
}
