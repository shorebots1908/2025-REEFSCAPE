package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean turnConnected = false;
    public boolean shootConnected = false;
    public double turnPositionRotations = 0.0;
    public double turnVelocityRPS = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTempCelsius = 0.0;
    public double shootVelocityRPS = 0.0;
    public double shootAppliedVolts = 0.0;
    public double shootCurrentAmps = 0.0;
    public double shootTempCelsius = 0.0;
  }

  public default void periodic() {}

  public default void updateInputs(TurretIOInputs inputs) {}

  public default String name() {
    return "Turret";
  }

  // Turn motor methods
  public default void setTurnPosition(double positionRotations) {}

  public default void setTurnOpenLoop(double output) {}

  public default void turnStop() {}

  public default double getTurnPosition() {
    return 0.0;
  }

  public default boolean isAtTurnTarget(double targetRotations, double toleranceRotations) {
    return Math.abs(getTurnPosition() - targetRotations) < toleranceRotations;
  }

  // Shoot motor methods
  public default void setShootVelocity(double velocityRPS) {}

  public default void setShootOpenLoop(double output) {}

  public default void shootStop() {}

  public default double getShootVelocity() {
    return 0.0;
  }
}
