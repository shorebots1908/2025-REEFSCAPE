package frc.robot.subsystems.wrist;

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

public class WristIOSparkMax implements WristIO {
  private final SparkMax wristMotor;
  private final SparkClosedLoopController wristController;
  private final SparkAbsoluteEncoder wristEncoder;
  private WristConfig config;
  private WristIOInputs inputs = new WristIOInputs();

  public WristIOSparkMax(WristConfig config) {
    this.config = config;
    inputs.targetEncoderPosition =
        config.startPosition.toRange(config.encoderLowerLimit, config.encoderUpperLimit);
    wristMotor = new SparkMax(config.wristMotorId, MotorType.kBrushless);
    wristEncoder = wristMotor.getAbsoluteEncoder();
    wristController = wristMotor.getClosedLoopController();

    // Wrist motor configuration
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .inverted(config.wristInvert)
        .apply(new AbsoluteEncoderConfig().inverted(config.encoderInvert))
        .closedLoopRampRate(config.rampRate)
        .apply(
            new ClosedLoopConfig()
                .p(config.pGain)
                .i(config.iGain)
                .d(config.dGain)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder))
        .apply(
            new SoftLimitConfig()
                .forwardSoftLimit(config.encoderUpperLimit)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(config.encoderLowerLimit)
                .reverseSoftLimitEnabled(true));
    wristMotor.configure(
        wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void periodic() {
    double position = inputs.positionRevs;
    double distance = inputs.targetEncoderPosition - inputs.positionRevs;
    double distanceAbsolute = Math.abs(distance);
    inputs.atTarget = distanceAbsolute < inputs.targetThreshold;
    BasePosition basePosition =
        BasePosition.fromRange(config.encoderLowerLimit, config.encoderUpperLimit, position);

    Logger.recordOutput(String.format("%s/WristEncoder", config.name), position);
    Logger.recordOutput(
        String.format("%s/WristBasePosition", config.name), basePosition.getValue());
    Logger.recordOutput(String.format("%s/DistanceToTarget", config.name), distanceAbsolute);
    Logger.recordOutput(String.format("%s/AtTarget", config.name), inputs.atTarget);
    Logger.recordOutput(
        String.format("%s/TargetEncoderPosition", config.name), inputs.targetEncoderPosition);
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void updateInputs(WristIO.WristIOInputs inputs) {
    ifOk(wristMotor, wristEncoder::getPosition, (value) -> inputs.positionRevs = value);
    this.inputs = inputs;
  }

  @Override
  public boolean atTargetPosition() {
    return inputs.atTarget;
  }

  @Override
  public void setWristOpenLoop(double output) {
    wristMotor.set(output);
    Logger.recordOutput(String.format("%s/OpenLoopOutput", config.name), output);
  }

  @Override
  public void setTargetPosition(BasePosition position) {
    inputs.targetEncoderPosition =
        position.toRange(config.encoderLowerLimit, config.encoderUpperLimit);
    wristController.setReference(inputs.targetEncoderPosition, ControlType.kPosition);
  }
}
