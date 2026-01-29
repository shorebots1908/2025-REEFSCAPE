package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
  private double targetPosition = 0.2; // Start at 10%

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput(io.name() + "/Position", inputs.position);
    Logger.recordOutput(io.name() + "/TargetPosition", targetPosition);
    io.periodic();

    Command active = this.getCurrentCommand();
    if (active != null) {
      Logger.recordOutput(io.name() + "/ActiveCommand", active.getName());
    }
  }

  /** Set position from 0.0 (retracted) to 1.0 (extended) */
  public void setPosition(double position) {
    targetPosition = Math.max(0.0, Math.min(1.0, position));
    io.setPosition(targetPosition);
  }

  /** Increase position by amount (clamped to 1.0 max) */
  public void increasePosition(double amount) {
    setPosition(targetPosition + amount);
  }

  /** Decrease position by amount (clamped to 0.0 min) */
  public void decreasePosition(double amount) {
    setPosition(targetPosition - amount);
  }

  public double getPosition() {
    return inputs.position;
  }

  public double getTargetPosition() {
    return targetPosition;
  }
}
