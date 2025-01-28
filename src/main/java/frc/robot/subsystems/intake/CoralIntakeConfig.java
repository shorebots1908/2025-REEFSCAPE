package frc.robot.subsystems.intake;

public class CoralIntakeConfig {
  public final double intakeSpeed = 0.6;
  public final double intakeReverse = -0.4;
  public final boolean intakeOn = false;

  // Required
  public final int leftMotorCanId;
  public final int rightMotorCanId;

  public CoralIntakeConfig(int left, int right) {
    this.leftMotorCanId = left;
    this.rightMotorCanId = right;
  }
}
