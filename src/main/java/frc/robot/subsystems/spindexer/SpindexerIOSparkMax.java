package frc.robot.subsystems.spindexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class SpindexerIOSparkMax implements SpindexerIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SpindexerConfig config;

  private SpindexerIOInputs inputs = new SpindexerIOInputs();

  public SpindexerIOSparkMax(SpindexerConfig config) {
    this.config = config;

    motor = new SparkMax(config.motorId, MotorType.kBrushless);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.currentLimit)
        .inverted(config.motorInvert);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(config.name + "/Velocity", inputs.velocityRPM);
    Logger.recordOutput(config.name + "/Current", inputs.currentAmps);
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.connected = true;
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();

    this.inputs = inputs;
  }

  @Override
  public void setOpenLoop(double output) {
    motor.set(output);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
