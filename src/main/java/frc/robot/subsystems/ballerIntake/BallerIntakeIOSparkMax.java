package frc.robot.subsystems.ballerIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class BallerIntakeIOSparkMax implements BallerIntakeIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  public BallerIntakeIOSparkMax(BallerIntakeConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
  }

  public void updateInputs(BallerIntakeIO.BallerIntakeIOInputs inputs) {
    // TODO
  }

  public void periodic() {}
}
