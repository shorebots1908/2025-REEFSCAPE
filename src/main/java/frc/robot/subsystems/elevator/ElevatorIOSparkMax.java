package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  public ElevatorIOSparkMax(ElevatorConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
  }

  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    // TODO
  }

  public void periodic() {}
}
