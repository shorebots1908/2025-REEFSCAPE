package frc.robot.subsystems.coralTool;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralToolIOSparkMax implements CoralToolIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax wristMotor;

  public CoralToolIOSparkMax(CoralToolConfig config) {
    // left motor is the leader
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    wristMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(config.leftMotorCanId, true);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
