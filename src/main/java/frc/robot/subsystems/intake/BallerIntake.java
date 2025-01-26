package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class BallerIntake {
  private SparkMax intake1 = new SparkMax(13, MotorType.kBrushless);
  private SparkMax intake2 = new SparkMax(14, MotorType.kBrushless);

  public BallerIntake() {}

  public void periodic() {}
  public void bintakeYes() {
    intake1.set(0.6);
    intake2.set(0.6);
  }
  public void bintakeNo() {
    intake1.set(-0.4);
    intake2.set(-0.4);
  }

}
