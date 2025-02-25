package frc.robot.subsystems.elevator;

public class ElevatorConfig {
  public final int leftMotorCanId;
  public final int rightMotorCanId;

  public ElevatorConfig(int leftMotorId, int rightMotorId) {
    this.leftMotorCanId = leftMotorId;
    this.rightMotorCanId = rightMotorId;
  }
}
