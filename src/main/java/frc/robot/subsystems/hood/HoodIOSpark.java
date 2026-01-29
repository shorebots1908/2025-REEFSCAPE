package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Servo;
import org.littletonrobotics.junction.Logger;

public class HoodIOSpark implements HoodIO {
  private final Servo actuator;
  private final HoodConfig config;
  private double currentPosition = 0.0;

  public HoodIOSpark(HoodConfig config) {
    this.config = config;
    actuator = new Servo(config.pwmChannel);
    setPosition(0.2); // Start at 10%
  }

  @Override
  public void periodic() {
    Logger.recordOutput(config.name + "/Position", currentPosition);
  }

  @Override
  public String name() {
    return config.name;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = currentPosition;
  }

  @Override
  public void setPosition(double position) {
    currentPosition = Math.max(0.0, Math.min(1.0, position));

    if (config.invert) {
      actuator.set(1.0 - currentPosition);
    } else {
      actuator.set(currentPosition);
    }
  }
}
