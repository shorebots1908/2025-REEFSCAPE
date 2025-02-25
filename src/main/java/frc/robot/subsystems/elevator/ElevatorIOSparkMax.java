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
  private double targetEncoderPosition = 0;
  private boolean atTarget = false;
  private double elevatorThreshold = 1.0;
  private double upperEncoderLimit = 63.7;
  private double lowerEncoderLimit = 0.0;

  public ElevatorIOSparkMax(ElevatorConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    controller = leftMotor.getClosedLoopController();
    limitUpper = leftMotor.getForwardLimitSwitch();
    limitLower = leftMotor.getReverseLimitSwitch();

    // Leader: Left motor config
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .apply(
            new SoftLimitConfig().forwardSoftLimit(upperEncoderLimit).forwardSoftLimitEnabled(true))
        .apply(
            new ClosedLoopConfig()
                .p(1.0)
                .i(0.0)
                .d(0.0)
                .apply(
                    new MAXMotionConfig()
                        .maxVelocity(1200.0)
                        .maxAcceleration(1200.0)
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
    double distance = targetEncoderPosition - inputs.positionRad;
    double distanceAbsolute = Math.abs(distance);
    atTarget = distanceAbsolute < elevatorThreshold;
    Logger.recordOutput("/Elevator/DistanceToTarget", distanceAbsolute);
    Logger.recordOutput("/Elevator/AtTarget", atTarget);

    controller.setReference(targetEncoderPosition, ControlType.kMAXMotionPositionControl);

    if (inputs.atLower) {
      leftMotor.getEncoder().setPosition(0);
    }

    if (inputs.atUpper) {
      upperEncoderLimit = leftEncoder.getPosition();
    }

    Logger.recordOutput(
        "/Elevator/BasePositionLower",
        new BasePosition(0).toRange(lowerEncoderLimit, upperEncoderLimit));
    Logger.recordOutput(
        "Elevator/BasePositionUpper",
        new BasePosition(1).toRange(lowerEncoderLimit, upperEncoderLimit));
  }

  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    inputs.atLower = limitLower.isPressed();
    inputs.atUpper = limitUpper.isPressed();
    this.inputs = inputs;
  }

  public void setTargetPosition(BasePosition position) {
    targetEncoderPosition = position.toRange(lowerEncoderLimit, upperEncoderLimit);
  }

  public boolean atTargetPosition() {
    return atTarget;
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
    leftMotor.setVoltage(output);
  }
}
