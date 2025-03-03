package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax wristMotor;
  private final SparkClosedLoopController wristController;
  private final SparkAbsoluteEncoder wristEncoder;
  private IntakeConfig config;
  private IntakeIO.IntakeIOInputs inputs = new IntakeIOInputs();

  // The target position the wrist will try to reach
  private double targetEncoderPosition = 0;

  // How close the wrist needs to be to be "close enough"
  private double targetThreshold = 0.01;

  // Whether the wrist is currently close enough
  private boolean atTarget = false;

  public IntakeIOSparkMax(IntakeConfig config) {
    this.config = config;
    targetEncoderPosition =
        config.startPosition.toRange(config.encoderLowerLimit, config.encoderUpperLimit);
    // Left wheel is leader
    leftMotor = new SparkMax(config.leftMotorId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorId, MotorType.kBrushless);
    wristMotor = new SparkMax(config.wristMotorId, MotorType.kBrushed);
    wristEncoder = wristMotor.getAbsoluteEncoder();
    wristController = wristMotor.getClosedLoopController();

    // Right wheel follows left wheel
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(config.leftMotorId, true);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Wrist motor configuration
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .apply(new AbsoluteEncoderConfig().inverted(config.encoderInvert))
        .closedLoopRampRate(1.0)
        .apply(
            new ClosedLoopConfig()
                .p(config.pGain)
                .i(config.iGain)
                .d(config.dGain)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder))
        .apply(
            new SoftLimitConfig()
                .forwardSoftLimit(config.encoderUpperLimit)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(config.encoderLowerLimit)
                .reverseSoftLimitEnabled(false));
    wristMotor.configure(
        wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void periodic() {
    double position = inputs.positionRevs;
    double distance = targetEncoderPosition - inputs.positionRevs;
    double distanceAbsolute = Math.abs(distance);
    atTarget = distanceAbsolute < targetThreshold;
    BasePosition basePosition =
        BasePosition.fromRange(config.encoderLowerLimit, config.encoderUpperLimit, position);

    Logger.recordOutput(String.format("%s/WristEncoder", config.name), position);
    Logger.recordOutput(
        String.format("%s/WristBasePosition", config.name), basePosition.getValue());
    Logger.recordOutput(String.format("%s/DistanceToTarget", config.name), distanceAbsolute);
    Logger.recordOutput(String.format("%s/AtTarget", config.name), atTarget);
    Logger.recordOutput(
        String.format("%s/TargetEncoderPosition", config.name), targetEncoderPosition);
  }

  public String name() {
    return config.name;
  }

  public void feedStop() {
    leftMotor.stopMotor();
  }

  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    // TODO
    ifOk(wristMotor, wristEncoder::getPosition, (value) -> inputs.positionRevs = value);

    this.inputs = inputs;
  }

  public boolean atTargetPosition() {
    return atTarget;
  }

  @Override
  public void setWristOpenLoop(double output) {
    wristMotor.set(output);
    Logger.recordOutput(String.format("%s/OpenLoopOutput", config.name), output);
  }

  @Override
  public void setFeedOpenLoop(double output) {
    leftMotor.set(output);
  }

  @Override
  public void setTargetPosition(BasePosition position) {
    targetEncoderPosition = position.toRange(config.encoderLowerLimit, config.encoderUpperLimit);
    wristController.setReference(targetEncoderPosition, ControlType.kPosition);
  }

  public boolean isHolding() {
    return false;
  }

  public boolean isEmpty() {
    return false;
  }
}
