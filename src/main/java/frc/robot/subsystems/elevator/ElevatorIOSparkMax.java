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
  private ElevatorIO.ElevatorIOInputs inputs = new ElevatorIOInputs();
  private double encoderUpperLimit = 0;
  private double encoderLowerLimit = 0;

  private SparkLimitSwitch limitUpper;
  private SparkLimitSwitch limitLower;

  LoggedNetworkNumber upperEncoderLimit = new LoggedNetworkNumber("/Tuning/Elevator/Upper", 50);
  LoggedNetworkNumber lowerEncoderLimit = new LoggedNetworkNumber("/Tuning/Elevator/Lower", 0);
  LoggedNetworkNumber elevatorP = new LoggedNetworkNumber("/Tuning/Elevator/P", 1);
  LoggedNetworkNumber elevatorI = new LoggedNetworkNumber("/Tuning/Elevator/I", 0);
  LoggedNetworkNumber elevatorD = new LoggedNetworkNumber("/Tuning/Elevator/D", 0);

  public ElevatorIOSparkMax(ElevatorConfig config) {
    leftMotor = new SparkMax(config.leftMotorCanId, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightMotor = new SparkMax(config.rightMotorCanId, MotorType.kBrushless);
    controller = leftMotor.getClosedLoopController();
    
    //left motor config
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.closedLoop.apply(new ClosedLoopConfig()
    .p(elevatorP.get())
    .i(elevatorI.get())
    .d(elevatorD.get()));
    leftMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //right motor config
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(9, true);
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
    double encoderTargetPosition = position.toRange(encoderLowerLimit, encoderUpperLimit);
    controller.setReference(encoderTargetPosition, ControlType.kPosition);
  }

  public void setElevatorOpenLoop(double output) {
    if(output > 0 && inputs.atUpper) {
      leftMotor.stopMotor();
      return;
    }
    if(output < 0 && inputs.atLower) {
      leftMotor.stopMotor();
      return;
    }
    leftMotor.setVoltage(output);
  }

  public void periodic() {
    if(inputs.atLower) {
      leftMotor.getEncoder().setPosition(0);
      lowerEncoderLimit.set(0);
    }

    if(inputs.atUpper) {
     upperEncoderLimit.set(leftEncoder.getPosition());
    }
  }
}
