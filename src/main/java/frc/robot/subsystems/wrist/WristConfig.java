package frc.robot.subsystems.wrist;

import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.subsystems.BasePosition;

public class WristConfig {
  public final String name;
  public final int wristMotorId;
  public final BasePosition startPosition;
  public final boolean isAbsoluteEncoder;
  public final SparkBaseConfig sparkConfig;

  public WristConfig(
      String name,
      int wristMotorId,
      BasePosition startPosition,
      boolean isAbsoluteEncoder,
      SparkBaseConfig sparkConfig) {
    this.name = name;
    this.wristMotorId = wristMotorId;
    this.startPosition = startPosition;
    this.isAbsoluteEncoder = isAbsoluteEncoder;
    this.sparkConfig = sparkConfig;
  }
}
