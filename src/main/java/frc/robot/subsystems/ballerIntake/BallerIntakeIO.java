package frc.robot.subsystems.ballerIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface BallerIntakeIO {
  @AutoLog
  public static class BallerIntakeIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(BallerIntakeIOInputs inputs) {}
}
