package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;

public class HoodCommands {
  public static final double STEP_SIZE = 0.1; // 10% per press

  /** Increase hood position by 10% */
  public static Command stepUp(Hood hood) {
    return stepUp(hood, STEP_SIZE);
  }

  /** Increase hood position by specified amount */
  public static Command stepUp(Hood hood, double amount) {
    return Commands.runOnce(() -> hood.increasePosition(amount), hood).withName("HoodStepUp");
  }

  /** Decrease hood position by 10% */
  public static Command stepDown(Hood hood) {
    return stepDown(hood, STEP_SIZE);
  }

  /** Decrease hood position by specified amount */
  public static Command stepDown(Hood hood, double amount) {
    return Commands.runOnce(() -> hood.decreasePosition(amount), hood).withName("HoodStepDown");
  }

  /** Move hood to fully retracted position (0%) */
  public static Command down(Hood hood) {
    return goToPosition(hood, 0.0).withName("HoodDown");
  }

  /** Move hood to fully extended position (100%) */
  public static Command up(Hood hood) {
    return goToPosition(hood, 1.0).withName("HoodUp");
  }

  /** Move hood to a specific position (0.0 to 1.0) */
  public static Command goToPosition(Hood hood, double position) {
    return Commands.runOnce(() -> hood.setPosition(position), hood).withName("HoodToPosition");
  }
}
