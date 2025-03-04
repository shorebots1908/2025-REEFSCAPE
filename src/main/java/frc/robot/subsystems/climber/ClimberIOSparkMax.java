package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSparkMax implements ClimberIO {
  private ClimberIO.ClimberIOInputs inputs = new ClimberIOInputs();
  private ClimberConfig config;
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private RelativeEncoder leftEncoder;
  private double targetEncoderPosition = 0;
  private double climberThreshold = 1.0;
  private boolean atTarget = false;
  public boolean isDeployed = false;
  private final SparkClosedLoopController controller;

  public ClimberIOSparkMax(ClimberConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    controller = leftMotor.getClosedLoopController();
    this.config = config;

    // Leader: Left motor config
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .apply(
            new ClosedLoopConfig()
                .p(config.pGain)
                .i(config.iGain)
                .d(config.dGain)
                .apply(
                    new MAXMotionConfig()
                        .maxVelocity(1200.0)
                        .maxAcceleration(1200.0)
                        .allowedClosedLoopError(0.3)));
    leftMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Follower: Right motor config
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(config.leftMotorCanId, true);
    followConfig.idleMode(IdleMode.kBrake);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void periodic() {
    double distance = targetEncoderPosition - inputs.positionTau;
    double distanceAbsolute = Math.abs(distance);
    atTarget = distanceAbsolute < climberThreshold;
    BasePosition basePosition =
        BasePosition.fromRange(
            config.encoderLowerLimit, config.encoderUpperLimit, inputs.positionTau);
    Logger.recordOutput("Climber/EncoderPosition", inputs.positionTau);
    Logger.recordOutput("Climber/BasePosition", basePosition.getValue());
  }

  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.positionTau = value);
    ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.velocityTauPerSec = value);
    this.inputs = inputs;
  }

  public void setOpenLoop(double output) {
    leftMotor.set(output);
  }

  public void setTargetPosition(BasePosition position) {
    targetEncoderPosition = position.toRange(config.encoderLowerLimit, config.encoderUpperLimit);
    controller.setReference(targetEncoderPosition, ControlType.kMAXMotionPositionControl);
  }

  public boolean atTargetPosition() {
    return atTarget;
  }

  public boolean isDeployed() {
    return isDeployed;
  }

  public void toggleDeploy() {
    isDeployed = !isDeployed;
  }

  public double getPosition() {
    return leftEncoder.getPosition();
  }
}
