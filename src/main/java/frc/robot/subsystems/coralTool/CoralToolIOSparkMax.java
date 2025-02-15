package frc.robot.subsystems.coralTool;

import static frc.robot.subsystems.drive.DriveConstants.driveBaseRadius;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;

import frc.robot.subsystems.BasePosition;

public class CoralToolIOSparkMax implements CoralToolIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax wristMotor;
  private final SparkClosedLoopController wristController;
  LoggedNetworkNumber coralP = new LoggedNetworkNumber("/tuning/coralP", 1.0);
  LoggedNetworkNumber coralI = new LoggedNetworkNumber("/tuning/coralI", 0);
  LoggedNetworkNumber coralD = new LoggedNetworkNumber("/tuning/coralD", 0);
  LoggedNetworkNumber coralLower = new LoggedNetworkNumber("/tuning/coralLower", 0);
  LoggedNetworkNumber coralUpper = new LoggedNetworkNumber("/tuning/coralUpper", 1);
  LoggedNetworkNumber coralMaxVelocity = new LoggedNetworkNumber("/tuning/coralVel", 5);
  LoggedNetworkNumber coralMaxAcceleration = new LoggedNetworkNumber("/tuning/coralAccel", 0.5);

  public CoralToolIOSparkMax(CoralToolConfig config) {
    // left motor is the leader
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    wristMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(config.leftMotorCanId, true);
    rightMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristController = wristMotor.getClosedLoopController();
    SparkMaxConfig motionConfig = new SparkMaxConfig();

    //Set PID gains
    motionConfig.closedLoop
      .p(coralP.get())
      .i(coralI.get())
      .d(coralD.get())
      .outputRange(-1, 1);
    
    //Set MAXMotion parameters
    motionConfig.closedLoop.maxMotion
      .maxVelocity(coralMaxVelocity.get())
      .maxAcceleration(coralMaxAcceleration.get());

    wristMotor.configure(motionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
  public void setWheelsOpenLoop(double output) {
    leftMotor.setVoltage(output);
  }

  @Override
  public void setPosition(BasePosition position){
    wristController.setReference(position.toRange(coralLower.get(),coralUpper.get()), SparkBase.ControlType.kMAXMotionPositionControl);
  }
}
