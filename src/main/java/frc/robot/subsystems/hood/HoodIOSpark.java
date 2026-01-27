package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.littletonrobotics.junction.Logger;

public class HoodIOSpark implements HoodIO {
  private final Spark motor;
  private final HoodConfig config;
  private double currentOutput = 0.0;

  public HoodIOSpark(HoodConfig config) {
    this.config = config;
    motor = new Spark(config.pwmChannel);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(config.name + "/AppliedOutput", currentOutput);
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedOutput = currentOutput;
  }

  @Override
  public void setOpenLoop(double output) {
    currentOutput = config.invert ? -output : output;
    motor.set(currentOutput);
  }

  @Override
  public void stop() {
    currentOutput = 0.0;
    motor.stopMotor();
  }
}
