package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private IntakeConfig config;
  private IntakeIO.IntakeIOInputs inputs = new IntakeIOInputs();
  private AnalogInput sensor;

  public IntakeIOSparkMax(IntakeConfig config) {
    this.config = config;
    // Left wheel is leader
    leftMotor = new SparkMax(config.leftMotorId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorId, MotorType.kBrushless);
    sensor = new AnalogInput(config.sensorId);

    // Left motor config
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right motor config
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(20);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void periodic() {
    Logger.recordOutput(String.format("%s/SensorValue", config.name), inputs.sensorValue);
    Logger.recordOutput(String.format("%s/SensorHolding", config.name), isHolding());
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void feedStop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    inputs.sensorValue = sensor.getValue();
    this.inputs = inputs;
  }

  @Override
  public void setFeedOpenLoop(double output) {
    leftMotor.set(output);
    rightMotor.set(output);
  }

  public boolean isHolding() {
    return inputs.sensorValue > config.sensorThreshold;
  }

  public boolean isEmpty() {
    return inputs.sensorValue < config.sensorThreshold;
  }
}
