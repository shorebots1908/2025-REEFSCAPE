package frc.robot.subsystems.climber;

import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {

    public double positionTau = 0;
    public double velocityTauPerSec = 0;
  }

  public default void periodic() {}

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setTargetPosition(BasePosition position) {}

  public default boolean atTargetPosition() {
    return false;
  }

  public default boolean isDeployed() {
    return false;
  }

  public default double getPosition() {
    return 0.0;
  }

  public default void toggleDeploy() {}

  public default void positionStop() {}
}
