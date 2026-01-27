// Before (SparkMax)
// Intake intake = new Intake(new IntakeIOSparkMax(intakeConfig));

// After (TalonFX/Kraken)
// Intake intake = new Intake(new IntakeIOTalonFX(intakeConfig));

// add the vendor library
// https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX leftMotor;
  private Optional<TalonFX> rightMotor = Optional.empty();
  private Optional<DigitalInput> digitalSensor = Optional.empty();
  private final IntakeConfig config;
  private IntakeIO.IntakeIOInputs inputs = new IntakeIOInputs();
  private final Timer timer;

  // Status signals for logging
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Current> leftSupplyCurrent;

  // Control request (reusable to avoid GC pressure)
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public IntakeIOTalonFX(IntakeConfig config) {
    this.config = config;
    timer = new Timer();

    // Initialize left motor (leader)
    leftMotor = new TalonFX(config.leftMotorId);

    // Configure left motor
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted =
        config.motorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 20;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.rampRate;

    leftMotor.getConfigurator().apply(leftConfig);

    // Cache status signals
    leftPosition = leftMotor.getPosition();
    leftVelocity = leftMotor.getVelocity();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(50, leftPosition, leftVelocity, leftSupplyCurrent);
    leftMotor.optimizeBusUtilization();

    // Initialize right motor if present
    if (config.rightMotorId.isPresent()) {
      var rightMotorId = config.rightMotorId.get();
      TalonFX right = new TalonFX(rightMotorId);

      TalonFXConfiguration rightConfig = new TalonFXConfiguration();
      rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      // Right motor inverted opposite to left
      rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      rightConfig.CurrentLimits.SupplyCurrentLimit = 20;
      rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      rightConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.rampRate;

      right.getConfigurator().apply(rightConfig);
      right.optimizeBusUtilization();

      rightMotor = Optional.of(right);
    }

    // Initialize digital sensor if present
    if (config.sensorId.isPresent()) {
      digitalSensor = Optional.of(new DigitalInput(config.sensorId.get()));
    }
  }

  @Override
  public void periodic() {
    // Refresh status signals
    BaseStatusSignal.refreshAll(leftPosition, leftVelocity, leftSupplyCurrent);

    Logger.recordOutput(String.format("%s/SensorValue", config.name), inputs.holdingSwitchPressed);
    Logger.recordOutput(String.format("%s/SensorHolding", config.name), isHolding());
    Logger.recordOutput(
        String.format("%s/LeftVelocityRPS", config.name), leftVelocity.getValueAsDouble());
    Logger.recordOutput(
        String.format("%s/LeftCurrentAmps", config.name), leftSupplyCurrent.getValueAsDouble());
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void feedStop() {
    leftMotor.stopMotor();
    rightMotor.ifPresent(TalonFX::stopMotor);
  }

  @Override
  public boolean timer() {
    if (timer.hasElapsed(0.25)) {
      leftMotor.stopMotor();
      timer.reset();
      timer.stop();
      return true;
    }
    return false;
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    inputs.connected = leftMotor.isConnected();
    inputs.positionRevs = leftPosition.getValueAsDouble();

    if (digitalSensor.isPresent()) {
      inputs.holdingSwitchPressed = digitalSensor.get().get();
    } else {
      inputs.holdingSwitchPressed = false;
    }
    this.inputs = inputs;
  }

  @Override
  public void setFeedOpenLoop(double output) {
    leftMotor.setControl(dutyCycleRequest.withOutput(output));
    rightMotor.ifPresent(m -> m.setControl(dutyCycleRequest.withOutput(output)));
  }

  @Override
  public void timerStart() {
    timer.start();
  }

  @Override
  public boolean isHolding() {
    timer.start();
    return inputs.holdingSwitchPressed;
  }
}
