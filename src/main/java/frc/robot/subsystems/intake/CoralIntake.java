package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralIntake {
  private SparkMax intake1 = new SparkMax(11, MotorType.kBrushless);
  private SparkMax intake2 = new SparkMax(12, MotorType.kBrushless);

  public CoralIntake() {}

  public void periodic() {}
  public void cintakeYes() {
    intake1.set(0.6);
    intake2.set(0.6);
  }
  public void cintakeNo() {
    intake1.set(-0.4);
    intake2.set(-0.4);
  }
  
}
