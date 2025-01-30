package frc.robot.subsystems.elevator;

public class ElevatorConfig {

  // Required
  public final int leftMotorCanId;
  public final int rightMotorCanId;

  public ElevatorConfig(int left, int right) {
    this.leftMotorCanId = left;
    this.rightMotorCanId = right;
  }
}
