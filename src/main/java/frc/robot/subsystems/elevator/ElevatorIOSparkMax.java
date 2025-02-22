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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorIOSparkMax implements ElevatorIO {
  // Left is leader
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkClosedLoopController controller;
  private ElevatorIO.ElevatorIOInputs inputs = new ElevatorIOInputs();
  private double targetEncoderPosition = 0;
  private boolean atTarget = false;

  private SparkLimitSwitch limitUpper;
  private SparkLimitSwitch limitLower;

  LoggedNetworkNumber upperEncoderLimit = new LoggedNetworkNumber("/Tuning/Elevator/Upper", 63.7);
  LoggedNetworkNumber lowerEncoderLimit = new LoggedNetworkNumber("/Tuning/Elevator/Lower", 0.0);
  LoggedNetworkNumber elevatorP = new LoggedNetworkNumber("/Tuning/Elevator/P", 5.0);
  LoggedNetworkNumber elevatorI = new LoggedNetworkNumber("/Tuning/Elevator/I", 0.0);
  LoggedNetworkNumber elevatorD = new LoggedNetworkNumber("/Tuning/Elevator/D", 0.0);
  LoggedNetworkNumber elevatorThreshold =
      new LoggedNetworkNumber("/Tuning/Elevator/Threshold", 1.0);
  LoggedNetworkNumber elevatorMaxVel = new LoggedNetworkNumber("/Tuning/Elevator/MaxVel", 100.0);
  LoggedNetworkNumber elevatorMaxAccel = new LoggedNetworkNumber("/Tuning/Elevator/MaxAccel", 10.0);

  public ElevatorIOSparkMax(ElevatorConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    controller = leftMotor.getClosedLoopController();

    // left motor config
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    // leaderConfig.limitSwitch.forwardLimitSwitchEnabled(true);
    // leaderConfig.limitSwitch.reverseLimitSwitchEnabled(true);
    leaderConfig.softLimit.forwardSoftLimitEnabled(true);
    leaderConfig.softLimit.forwardSoftLimit(upperEncoderLimit.get() - 1.0);
    leaderConfig.idleMode(IdleMode.kBrake);

    leaderConfig.closedLoop.apply(
        new ClosedLoopConfig()
            .p(elevatorP.get())
            .i(elevatorI.get())
            .d(elevatorD.get())
            .apply(
                new MAXMotionConfig()
                    .maxVelocity(elevatorMaxVel.get())
                    .maxAcceleration(elevatorMaxAccel.get())));

    // leaderConfig
    //     .closedLoop
    //     .maxMotion
    //     .maxVelocity(elevatorMaxVel.get())
    //     .maxAcceleration(elevatorMaxAccel.get());
    leftMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // right motor config
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(9, true);
    followConfig.idleMode(IdleMode.kBrake);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    limitUpper = leftMotor.getForwardLimitSwitch();
    limitLower = leftMotor.getReverseLimitSwitch();
  }

  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    inputs.atLower = limitLower.isPressed();
    inputs.atUpper = limitUpper.isPressed();
    this.inputs = inputs;
  }

  public void setTargetPosition(BasePosition position) {
    targetEncoderPosition = position.toRange(lowerEncoderLimit.get(), upperEncoderLimit.get());
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

  @Override
  public void periodic() {
    double distance = targetEncoderPosition - inputs.positionRad;
    double distanceAbsolute = Math.abs(distance);
    atTarget = distanceAbsolute < elevatorThreshold.get();
    Logger.recordOutput("/Elevator/DistanceToTarget", distanceAbsolute);
    Logger.recordOutput("/Elevator/AtTarget", atTarget);

    controller.setReference(targetEncoderPosition, ControlType.kMAXMotionPositionControl);

    if (inputs.atLower) {
      leftMotor.getEncoder().setPosition(0);
      lowerEncoderLimit.set(0);
    }

    if (inputs.atUpper) {
      upperEncoderLimit.set(leftEncoder.getPosition());
    }

    Logger.recordOutput(
        "/Elevator/BasePositionLower",
        new BasePosition(0).toRange(lowerEncoderLimit.get(), upperEncoderLimit.get()));
    Logger.recordOutput(
        "Elevator/BasePositionUpper",
        new BasePosition(1).toRange(lowerEncoderLimit.get(), upperEncoderLimit.get()));
  }
}
