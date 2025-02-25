package frc.robot.subsystems.coralTool;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.AutoLog;

public interface CoralToolIO {
  @AutoLog
  public static class CoralToolIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public double positionRevs = 0.0;
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void periodic() {}

  public default void updateInputs(CoralToolIOInputs inputs) {}

  public default void setTargetPosition(BasePosition position) {}

  public default boolean atTargetPosition() {
    return false;
  }

  public default void setWristOpenLoop(double output) {}

  public default void setFeedOpenLoop(double output) {}

  public default void feedStop() {}

  public default boolean isHolding() {
    return false;
  }

  public default boolean isEmpty() {
    return false;
  }
}
