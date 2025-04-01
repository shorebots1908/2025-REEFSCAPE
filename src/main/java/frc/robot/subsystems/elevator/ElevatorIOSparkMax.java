package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSparkMax implements ElevatorIO {
  // Left is leader
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkClosedLoopController controller;
  private final SparkLimitSwitch limitUpper;
  private final SparkLimitSwitch limitLower;

  private ElevatorIO.ElevatorIOInputs inputs = new ElevatorIOInputs();

  private ElevatorConfig config;

  public ElevatorIOSparkMax(ElevatorConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    controller = leftMotor.getClosedLoopController();
    limitUpper = leftMotor.getForwardLimitSwitch();
    limitLower = leftMotor.getReverseLimitSwitch();
    this.config = config;

    // Leader: Left motor config
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .apply(
            new SoftLimitConfig()
                .forwardSoftLimit(config.encoderUpperLimit - 0.5)
                .forwardSoftLimitEnabled(true))
        .apply(
            new ClosedLoopConfig()
                .p(config.pGain)
                .i(config.iGain)
                .d(config.dGain)
                .apply(
                    new MAXMotionConfig()
                        .maxVelocity(5600.0) // was 3600
                        .maxAcceleration(4800.0)
                        .allowedClosedLoopError(0.3)));
    leftMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Follower: Right motor config
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(9, true);
    followConfig.idleMode(IdleMode.kBrake);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    double distance = inputs.targetEncoderPosition - inputs.positionRad;
    double distanceAbsolute = Math.abs(distance);
    inputs.atTarget = distanceAbsolute < inputs.elevatorThreshold;
    BasePosition basePosition =
        BasePosition.fromRange(
            config.encoderLowerLimit, config.encoderUpperLimit, inputs.positionRad);
    Logger.recordOutput("Elevator/BasePosition", basePosition.getValue());
    Logger.recordOutput("Elevator/DistanceToTarget", distanceAbsolute);
    Logger.recordOutput("Elevator/AtTarget", inputs.atTarget);

    Logger.recordOutput(
        "Elevator/BasePositionLower",
        new BasePosition(0).toRange(config.encoderLowerLimit, config.encoderUpperLimit));
    Logger.recordOutput(
        "Elevator/BasePositionUpper",
        new BasePosition(1).toRange(config.encoderLowerLimit, config.encoderUpperLimit));
    Logger.recordOutput("Elevator/EncoderPosition", inputs.positionRad);
  }

  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    inputs.atLower = limitLower.isPressed();
    inputs.atUpper = limitUpper.isPressed();
    this.inputs = inputs;
  }

  public void setTargetPosition(BasePosition position) {
    inputs.targetEncoderPosition =
        position.toRange(config.encoderLowerLimit, config.encoderUpperLimit);
    controller.setReference(inputs.targetEncoderPosition, ControlType.kMAXMotionPositionControl);
  }

  public boolean atTargetPosition() {
    return inputs.atTarget;
  }

  public void setElevatorOpenLoop(double output) {
    if (output > 0 && inputs.atUpper) {
      leftMotor.stopMotor();
      return;
    }
    if (output < 0 && inputs.atLower) {
      leftMotor.stopMotor();
      return;
    }
    leftMotor.set(output);
  }

  @Override
  public ElevatorConfig getConfig() {
    return this.config;
  }

  @Override
  public RelativeEncoder getEncoder() {
    return leftEncoder;
  }
}
