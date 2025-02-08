package frc.robot.subsystems.coralIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralIntakeIOSparkMax implements CoralIntakeIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  public CoralIntakeIOSparkMax(CoralIntakeConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
  }

  public void updateInputs(CoralIntakeIO.CoralIntakeIOInputs inputs) {
    // TODO
  }

  public void periodic() {}
}
