package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax leftMotor;
  private Optional<SparkMax> rightMotor = Optional.empty();
  private Optional<DigitalInput> digitalSensor = Optional.empty();
  private IntakeConfig config;
  private IntakeIO.IntakeIOInputs inputs = new IntakeIOInputs();

  public IntakeIOSparkMax(IntakeConfig config) {
    this.config = config;
    // Left wheel is leader
    leftMotor = new SparkMax(config.leftMotorId, MotorType.kBrushless);
    // Left motor config
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).inverted(config.motorInvert);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (config.rightMotorId.isPresent()) {
      var rightMotorId = config.rightMotorId.get();
      rightMotor = Optional.of(new SparkMax(rightMotorId, MotorType.kBrushless));
      // Right motor config
      SparkMaxConfig rightConfig = new SparkMaxConfig();
      rightConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(20);
      rightMotor
          .get()
          .configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    if (config.sensorId.isPresent()) {
      digitalSensor = Optional.of(new DigitalInput(config.sensorId.get()));
    }
  }

  public void periodic() {
    // if (digitalSensor.isPresent()) {
    //   inputs.holdingSwitchPressed = digitalSensor.get().get();
    // }
    Logger.recordOutput(String.format("%s/SensorValue", config.name), inputs.holdingSwitchPressed);
    Logger.recordOutput(String.format("%s/SensorHolding", config.name), isHolding());
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void feedStop() {
    leftMotor.stopMotor();
    if (rightMotor.isPresent()) {
      rightMotor.get().stopMotor();
    }
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    if (digitalSensor.isPresent()) {
      inputs.holdingSwitchPressed = digitalSensor.get().get();
    } else {
      inputs.holdingSwitchPressed = false;
    }
    this.inputs = inputs;
  }

  @Override
  public void setFeedOpenLoop(double output) {
    leftMotor.set(output);
    if (rightMotor.isPresent()) {
      rightMotor.get().set(output);
    }
  }

  public boolean isHolding() {
    // return inputs.analogSensorValue > config.sensorThreshold;

    return inputs.holdingSwitchPressed;
  }
}
