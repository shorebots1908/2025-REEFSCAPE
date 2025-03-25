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

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSparkMax implements ClimberIO {
  private ClimberIO.ClimberIOInputs inputs = new ClimberIOInputs();
  private ClimberConfig config;
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private RelativeEncoder leftEncoder;
  private double climberThreshold = 1.0;
  private boolean atTarget = false;
  public boolean isDeployed = false;
  private final SparkClosedLoopController controller;
  public Rotation2d targetEncoderRotation = new Rotation2d();

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
    Rotation2d distance = Rotation2d.fromRotations(targetEncoderRotation.getRotations() - inputs.rotation.getRotations());
    double distanceAbsolute = Math.abs(distance.getRotations());
    atTarget = distanceAbsolute < climberThreshold;
    BasePosition basePosition =
        BasePosition.fromRange(
            config.encoderLowerLimit.getRotations(), 
            config.encoderUpperLimit.getRotations(), 
            inputs.rotation.getRotations()
        );
    Logger.recordOutput("Climber/EncoderPosition", inputs.rotation);
    Logger.recordOutput("Climber/BasePosition", basePosition.getValue());
  }

  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.rotation = Rotation2d.fromRotations(value));
    this.inputs = inputs;
  }

  public void setOpenLoop(double output) {
    leftMotor.set(output);
  }

  public void setTargetPosition(BasePosition position) {
    targetEncoderRotation = Rotation2d.fromRotations(position.toRange(
      config.encoderLowerLimit.getRotations(),
      config.encoderUpperLimit.getRotations()
    ));
    controller.setReference(targetEncoderRotation.getRotations(), ControlType.kMAXMotionPositionControl);
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromRotations(leftEncoder.getPosition());
  }
}
