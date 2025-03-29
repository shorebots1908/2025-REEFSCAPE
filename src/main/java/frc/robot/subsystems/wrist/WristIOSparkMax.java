package frc.robot.subsystems.wrist;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.BasePosition;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class WristIOSparkMax implements WristIO {
  private final SparkMax wristMotor;
  private final SparkClosedLoopController wristController;
  private final Optional<SparkAbsoluteEncoder> wristAbsoluteEncoder;
  private final Optional<RelativeEncoder> wristRelativeEncoder;
  private WristConfig config;
  private WristIOInputs inputs = new WristIOInputs();

  public WristIOSparkMax(WristConfig config) {
    this.config = config;
    inputs.targetEncoderPosition = config.startPosition.toRange(0.58, 3.1);
    wristMotor = new SparkMax(config.wristMotorId, MotorType.kBrushless);
    if (config.isAbsoluteEncoder) {
      wristAbsoluteEncoder = Optional.of(wristMotor.getAbsoluteEncoder());
      wristRelativeEncoder = Optional.empty();
    } else {
      wristRelativeEncoder = Optional.of(wristMotor.getEncoder());
      wristAbsoluteEncoder = Optional.empty();
    }
    wristController = wristMotor.getClosedLoopController();

    wristMotor.configure(
        config.sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void periodic() {
    double position = inputs.positionRevs;
    double distance = inputs.targetEncoderPosition - inputs.positionRevs;
    double distanceAbsolute = Math.abs(distance);
    inputs.atTarget = distanceAbsolute < inputs.targetThreshold;
    BasePosition basePosition = BasePosition.fromRange(0.58, 3.1, position);

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
    if (config.isAbsoluteEncoder) {
      ifOk(
          wristMotor,
          () -> wristAbsoluteEncoder.get().getPosition(),
          (value) -> inputs.positionRevs = value);
    } else {
      ifOk(
          wristMotor,
          () -> wristRelativeEncoder.get().getPosition(),
          (value) -> inputs.positionRevs = value);
    }
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
    inputs.targetEncoderPosition = position.toRange(config.lowerLimit, config.upperLimit);
    wristController.setReference(inputs.targetEncoderPosition, ControlType.kPosition);
  }
}
