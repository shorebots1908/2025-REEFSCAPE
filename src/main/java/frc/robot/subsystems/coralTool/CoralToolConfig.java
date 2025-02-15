package frc.robot.subsystems.coralTool;

public class CoralToolConfig {
  public final double intakeSpeed = 0.6;
  public final double intakeReverse = -0.4;
  public final boolean intakeOn = false;
  public final double wristTop = 0.15;
  public final double wristBottom = -0.15;

  // Required
  public final int leftMotorCanId;
  public final int rightMotorCanId;
  public final int wristMotorCanId;

  public CoralToolConfig(int left, int right, int wrist) {
    this.leftMotorCanId = left;
    this.rightMotorCanId = right;
    this.wristMotorCanId = wrist;
  }
}
