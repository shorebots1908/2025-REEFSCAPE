package frc.robot.subsystems.coralTool;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.BasePosition;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

public interface CoralToolIO {
  @AutoLog
  public static class CoralToolIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(CoralToolIOInputs inputs) {}

  public default void setWristOpenLoop(double output) {}

  public default void setWheelsOpenLoop(double output) {}

  public default void setPosition(BasePosition position) {}
}
