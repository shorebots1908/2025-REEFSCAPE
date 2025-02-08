package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorIOSparkMax implements ElevatorIO {
  // Left is leader
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkClosedLoopController controller;
  private double encoderUpperLimit = 0;
  private double encoderLowerLimit = 0;

  private LoggedNetworkNumber elevatorUpperLimit;
  private LoggedNetworkNumber elevatorLowerLimit;

  private DigitalInput limitUpper;
  private DigitalInput limitLower;

  public ElevatorIOSparkMax(ElevatorConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    controller = leftMotor.getClosedLoopController();
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(9, true);
    rightMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    limitUpper = new DigitalInput(config.upperLimitId);
    limitLower = new DigitalInput(config.lowerLimitId);

    elevatorUpperLimit =
        new LoggedNetworkNumber("/Tuning/ElevatorUpperValue", config.upperEncoderValue);
    elevatorLowerLimit =
        new LoggedNetworkNumber("/Tuning/ElevatorLowerValue", config.lowerEncoderValue);
  }

  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    inputs.atLower = limitLower.get();
    inputs.atUpper = limitUpper.get();
  }

  public void setTargetPosition(BasePosition position) {
    double encoderTargetPosition = position.toRange(encoderLowerLimit, encoderUpperLimit);
    controller.setReference(encoderTargetPosition, ControlType.kPosition);

    double speed = 0.5;
    controller.setReference(speed, ControlType.kVelocity);
  }

  public void setElevatorOpenLoop(double output) {
    leftMotor.setVoltage(output);
  }

  public boolean atLowerLimit() {
    return limitLower.get();
  }

  public boolean atUpperLimit() {
    return limitUpper.get();
  }

  public void periodic() {}
}
