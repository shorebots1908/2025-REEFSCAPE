package frc.robot.subsystems.elevator;

import frc.robot.subsystems.BasePosition;

public interface ElevatorIO {
  public static class ElevatorIOInputs {
    public boolean connected = false;
    public BasePosition position = new BasePosition(0);
    public boolean atTop = false;
    public boolean atBottom = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {
    
  }
}
