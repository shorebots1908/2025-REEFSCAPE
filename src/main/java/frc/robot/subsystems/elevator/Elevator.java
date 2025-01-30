package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * TODO:
 *
 * <p>- [ ] Make ElevatorIO an interface - [ ] Add ElevatorIOSpark class - [ ] Move SparkMax classes
 * to ElevatorIOSpark -
 */
public class Elevator {
  private SparkMax leadElevator = new SparkMax(9, MotorType.kBrushless);
  private SparkMax followElevator = new SparkMax(10, MotorType.kBrushless);

  public Elevator() {
    SparkMaxConfig followConfig = new SparkMaxConfig();
    followConfig.follow(9, true);
    followElevator.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void elevatorMove() {}
}
