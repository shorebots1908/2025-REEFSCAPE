package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class KickerIOTalonFX implements KickerIO {
  private final TalonFX motor;
  private final KickerConfig config;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temp;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

  private FeederIOInputs inputs = new FeederIOInputs();

  public KickerIOTalonFX(KickerConfig config) {
    this.config = config;

    motor = new TalonFX(config.motorId);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted =
        config.motorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = config.currentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor.getConfigurator().apply(motorConfig);

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current, temp);
    motor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(config.name + "/Velocity", inputs.velocityRPS);
    Logger.recordOutput(config.name + "/Current", inputs.currentAmps);
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, appliedVolts, current, temp);

    inputs.connected = motor.isAlive();
    inputs.velocityRPS = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    this.inputs = inputs;
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setControl(dutyCycle.withOutput(output));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
