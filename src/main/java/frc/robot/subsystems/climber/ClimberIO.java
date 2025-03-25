package frc.robot.subsystems.climber;

import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public Rotation2d rotation = new Rotation2d();
  }

  public default void periodic() {}

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setTargetPosition(BasePosition position) {}

  public default BasePosition getTargetPosition() {
    return new BasePosition(0.0);
  }

  public default Rotation2d getRotation() {
    return new Rotation2d();
  }

  public default void positionStop() {}
}
