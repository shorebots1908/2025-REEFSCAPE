package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ElevatorIO {
  public static class ElevatorIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
