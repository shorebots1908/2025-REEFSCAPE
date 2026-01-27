package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.kicker.Kicker;
import java.util.function.DoubleSupplier;

public class KickerCommands {
  public static final double DEFAULT_SPEED = 0.3; // 30% speed

  // ==================== BASIC COMMANDS ====================

  /** Run feeder forward at default speed */
  public static Command forward(Kicker feeder) {
    return forward(feeder, DEFAULT_SPEED);
  }

  /** Run feeder forward at specified speed */
  public static Command forward(Kicker feeder, double speed) {
    double output = Math.abs(speed);
    return Commands.run(() -> feeder.setOpenLoop(output), feeder)
        .finallyDo(() -> feeder.stop())
        .withName("FeederForward");
  }

  /** Run feeder reverse at default speed */
  public static Command reverse(Kicker feeder) {
    return reverse(feeder, DEFAULT_SPEED);
  }

  /** Run feeder reverse at specified speed */
  public static Command reverse(Kicker feeder, double speed) {
    double output = -Math.abs(speed);
    return Commands.run(() -> feeder.setOpenLoop(output), feeder)
        .finallyDo(() -> feeder.stop())
        .withName("FeederReverse");
  }

  /** Manual control with joystick */
  public static Command manual(Kicker feeder, DoubleSupplier speed) {
    return Commands.run(() -> feeder.setOpenLoop(speed.getAsDouble()), feeder)
        .finallyDo(() -> feeder.stop())
        .withName("FeederManual");
  }

  /** Stop feeder */
  public static Command stop(Kicker feeder) {
    return Commands.runOnce(feeder::stop, feeder).withName("FeederStop");
  }
}
