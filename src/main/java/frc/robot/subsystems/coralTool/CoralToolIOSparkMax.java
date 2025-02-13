package frc.robot.subsystems.coralTool;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralToolIOSparkMax implements CoralToolIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax wristMotor;

  public CoralToolIOSparkMax(CoralToolConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    wristMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
  }

  public void updateInputs(CoralToolIO.CoralToolIOInputs inputs) {
    // TODO
  }

  public void periodic() {}

  @Override
  public void setWristOpenLoop(double output) {
    wristMotor.setVoltage(output);
  }

  @Override
  public void setWheelsOpenLoop(double output) {
    leftMotor.setVoltage(output);
  }
}
