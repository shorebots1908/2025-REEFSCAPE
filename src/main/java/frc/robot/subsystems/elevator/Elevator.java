package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.BasePosition;

/**
 * TODO:
 *
 * <p>- [ ] Make ElevatorIO an interface - [ ] Add ElevatorIOSpark class - [ ] Move SparkMax classes
 * to ElevatorIOSpark -
 */
public class Elevator {
  

  public Elevator() {
    
  }

  public void goToPosition(BasePosition position) {}

  public void positionStop() {}

  public boolean limitTop() {

    return false;
  }
  public boolean limitBottom() {

    return false;
  }
  public double getPosition() {
    
    return 0.0;
  }
}
