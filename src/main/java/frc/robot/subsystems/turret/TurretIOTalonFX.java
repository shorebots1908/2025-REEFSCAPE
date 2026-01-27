package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turnMotor;
  private final TalonFX shootMotor;
  private final TurretConfig config;

  // Turn motor signals
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;
  private final StatusSignal<Temperature> turnTemp;

  // Shoot motor signals
  private final StatusSignal<AngularVelocity> shootVelocity;
  private final StatusSignal<Voltage> shootAppliedVolts;
  private final StatusSignal<Current> shootCurrent;
  private final StatusSignal<Temperature> shootTemp;

  // Control requests
  private final DutyCycleOut turnDutyCycle = new DutyCycleOut(0);
  private final DutyCycleOut shootDutyCycle = new DutyCycleOut(0);
  private final PositionVoltage turnPositionControl = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage shootVelocityControl = new VelocityVoltage(0).withSlot(0);

  private TurretIOInputs inputs = new TurretIOInputs();

  public TurretIOTalonFX(TurretConfig config) {
    this.config = config;

    turnMotor = new TalonFX(config.turnMotorId);
    shootMotor = new TalonFX(config.shootMotorId);

    // Configure turn motor
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted =
        config.turnMotorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.Slot0.kP = config.turnPGain;
    turnConfig.Slot0.kI = config.turnIGain;
    turnConfig.Slot0.kD = config.turnDGain;
    turnConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnMotor.getConfigurator().apply(turnConfig);

    // Configure shoot motor
    TalonFXConfiguration shootConfig = new TalonFXConfiguration();
    shootConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootConfig.MotorOutput.Inverted =
        config.shootMotorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    shootConfig.Slot0.kP = 0.1;
    shootConfig.Slot0.kI = 0.0;
    shootConfig.Slot0.kD = 0.0;
    shootConfig.Slot0.kV = 0.12;
    shootConfig.CurrentLimits.SupplyCurrentLimit = 60;
    shootConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shootMotor.getConfigurator().apply(shootConfig);

    // Get status signals for turn motor
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getSupplyCurrent();
    turnTemp = turnMotor.getDeviceTemp();

    // Get status signals for shoot motor
    shootVelocity = shootMotor.getVelocity();
    shootAppliedVolts = shootMotor.getMotorVoltage();
    shootCurrent = shootMotor.getSupplyCurrent();
    shootTemp = shootMotor.getDeviceTemp();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTemp,
        shootVelocity,
        shootAppliedVolts,
        shootCurrent,
        shootTemp);

    turnMotor.optimizeBusUtilization();
    shootMotor.optimizeBusUtilization();

    // Reset turn position to home
    turnMotor.setPosition(config.homeRotations);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(config.name + "/TurnPosition", inputs.turnPositionRotations);
    Logger.recordOutput(config.name + "/TurnVelocity", inputs.turnVelocityRPS);
    Logger.recordOutput(config.name + "/ShootVelocity", inputs.shootVelocityRPS);
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTemp,
        shootVelocity,
        shootAppliedVolts,
        shootCurrent,
        shootTemp);

    inputs.turnConnected = turnMotor.isAlive();
    inputs.turnPositionRotations = turnPosition.getValueAsDouble();
    inputs.turnVelocityRPS = turnVelocity.getValueAsDouble();
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    inputs.turnTempCelsius = turnTemp.getValueAsDouble();

    inputs.shootConnected = shootMotor.isAlive();
    inputs.shootVelocityRPS = shootVelocity.getValueAsDouble();
    inputs.shootAppliedVolts = shootAppliedVolts.getValueAsDouble();
    inputs.shootCurrentAmps = shootCurrent.getValueAsDouble();
    inputs.shootTempCelsius = shootTemp.getValueAsDouble();

    this.inputs = inputs;
  }

  // Turn motor methods
  @Override
  public void setTurnPosition(double positionRotations) {
    turnMotor.setControl(turnPositionControl.withPosition(positionRotations));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnMotor.setControl(turnDutyCycle.withOutput(output));
  }

  @Override
  public void turnStop() {
    turnMotor.stopMotor();
  }

  @Override
  public double getTurnPosition() {
    return inputs.turnPositionRotations;
  }

  // Shoot motor methods
  @Override
  public void setShootVelocity(double velocityRPS) {
    shootMotor.setControl(shootVelocityControl.withVelocity(velocityRPS));
  }

  @Override
  public void setShootOpenLoop(double output) {
    shootMotor.setControl(shootDutyCycle.withOutput(output));
  }

  @Override
  public void shootStop() {
    shootMotor.stopMotor();
  }

  @Override
  public double getShootVelocity() {
    return inputs.shootVelocityRPS;
  }
}
