package frc.robot.subsystems.coralTool;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class CoralToolIOSparkMax implements CoralToolIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax wristMotor;
  private final SparkClosedLoopController wristController;
  private final SparkAbsoluteEncoder coralEncoder;
  private CoralToolIO.CoralToolIOInputs inputs = new CoralToolIOInputs();
  private double limitLower = 0.0;
  private double limitUpper = 0.375;
  private double targetEncoderPosition = 0;
  private double coralThreshold = 1.0;
  private boolean atTarget = false;
  LoggedNetworkNumber coralP = new LoggedNetworkNumber("/Tuning/Coral/P", 2.0);
  LoggedNetworkNumber coralI = new LoggedNetworkNumber("/Tuning/Coral/I", 0);
  LoggedNetworkNumber coralD = new LoggedNetworkNumber("/Tuning/Coral/D", 0);
  LoggedNetworkNumber coralMaxVelocity = new LoggedNetworkNumber("/Tuning/Coral/Vel", 1000);
  LoggedNetworkNumber coralMaxAcceleration = new LoggedNetworkNumber("/Tuning/Coral/Accel", 60);

  public CoralToolIOSparkMax(CoralToolConfig config) {
    // left motor is the leader
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    wristMotor = new SparkMax(config.wristMotorCanId, MotorType.kBrushed);
    coralEncoder = wristMotor.getAbsoluteEncoder();
    SparkMaxConfig followConfig = new SparkMaxConfig();

    followConfig.follow(config.leftMotorCanId, true);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristController = wristMotor.getClosedLoopController();
    SparkMaxConfig wristConfig = new SparkMaxConfig();

    // Set PID gains
    wristConfig.closedLoop.p(coralP.get()).i(coralI.get()).d(coralD.get());

    wristConfig.closedLoopRampRate(0.5);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristConfig.apply(
        new SoftLimitConfig().forwardSoftLimit(limitLower).reverseSoftLimit(limitUpper));
    wristMotor.configure(
        wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(CoralToolIO.CoralToolIOInputs inputs) {
    // TODO
  }

  public void periodic() {
    double position = coralEncoder.getPosition();
    Logger.recordOutput("/Coral/CoralWristEncoder", position);
    double distance = targetEncoderPosition - inputs.positionRevs;
    double distanceAbsolute = Math.abs(distance);
    atTarget = distanceAbsolute < coralThreshold;
    Logger.recordOutput("/Elevator/DistanceToTarget", distanceAbsolute);
    Logger.recordOutput("/Elevator/AtTarget", atTarget);

    wristController.setReference(targetEncoderPosition, ControlType.kPosition);
  }

  public boolean atTargetPosition() {
    return atTarget;
  }

  @Override
  public void setWristOpenLoop(double output) {
    wristMotor.setVoltage(output * 12);
  }

  @Override
  public void setFeedOpenLoop(double output) {
    leftMotor.setVoltage(output);
  }

  @Override
  public void setTargetPosition(BasePosition position) {
    targetEncoderPosition = position.toRange(limitLower, limitUpper);
  }

  public boolean isHolding() {

    return false;
  }

  public boolean isEmpty() {
    return false;
  }
}
