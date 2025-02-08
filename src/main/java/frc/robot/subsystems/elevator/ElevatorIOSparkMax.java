package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.BasePosition;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private double encoderUpperLimit = 0;
  private double encoderLowerLimit = 0;


  public ElevatorIOSparkMax(ElevatorConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(9, true);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  
  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    // TODO
    inputs.position = BasePosition.fromRange(encoderLowerLimit, encoderUpperLimit, leftMotor.getEncoder().getPosition());
  }

  public void periodic() {}
}
