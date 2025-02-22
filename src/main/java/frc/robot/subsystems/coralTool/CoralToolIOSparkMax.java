package frc.robot.subsystems.coralTool;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class CoralToolIOSparkMax implements CoralToolIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax wristMotor;
  private final SparkClosedLoopController wristController;
  LoggedNetworkNumber coralP = new LoggedNetworkNumber("/Tuning/Coral/P", 1.0);
  LoggedNetworkNumber coralI = new LoggedNetworkNumber("/Tuning/Coral/I", 0);
  LoggedNetworkNumber coralD = new LoggedNetworkNumber("/Tuning/Coral/D", 0);
  LoggedNetworkNumber coralLower = new LoggedNetworkNumber("/Tuning/Coral/Lower", 0);
  LoggedNetworkNumber coralUpper = new LoggedNetworkNumber("/Tuning/Coral/Upper", 1);
  LoggedNetworkNumber coralMaxVelocity = new LoggedNetworkNumber("/Tuning/Coral/Vel", 5);
  LoggedNetworkNumber coralMaxAcceleration = new LoggedNetworkNumber("/Tuning/Coral/Accel", 0.5);

  public CoralToolIOSparkMax(CoralToolConfig config) {
    // left motor is the leader
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    wristMotor = new SparkMax(config.wristMotorCanId, MotorType.kBrushed);
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(config.leftMotorCanId, true);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristController = wristMotor.getClosedLoopController();
    SparkMaxConfig motionConfig = new SparkMaxConfig();

    // Set PID gains
    motionConfig.closedLoop.p(coralP.get()).i(coralI.get()).d(coralD.get()).outputRange(-1, 1);

    // Set MAXMotion parameters
    motionConfig
        .closedLoop
        .maxMotion
        .maxVelocity(coralMaxVelocity.get())
        .maxAcceleration(coralMaxAcceleration.get());

    wristMotor.configure(
        motionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(CoralToolIO.CoralToolIOInputs inputs) {
    // TODO
  }

  public void periodic() {}

  @Override
  public void setWristOpenLoop(double output) {
    wristMotor.setVoltage(output);
  }

  @Override
  public void setFeedOpenLoop(double output) {
    leftMotor.setVoltage(output);
  }

  @Override
  public void setWristPosition(BasePosition position) {
    wristController.setReference(
        position.toRange(coralLower.get(), coralUpper.get()), SparkBase.ControlType.kPosition);
  }

  public boolean isHolding() {

    return false;
  }

  public boolean isEmpty() {
    return false;
  }
}
