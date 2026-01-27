package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import java.util.function.DoubleSupplier;

public class HoodCommands {
  public static final double DEFAULT_SPEED = 0.3; // 30% speed

  /** Move hood up at default speed */
  public static Command up(Hood hood) {
    return up(hood, DEFAULT_SPEED);
  }

  /** Move hood up at specified speed */
  public static Command up(Hood hood, double speed) {
    double output = Math.abs(speed);
    return Commands.run(() -> hood.setOpenLoop(output), hood)
        .finallyDo(() -> hood.stop())
        .withName("HoodUp");
  }

  /** Move hood down at default speed */
  public static Command down(Hood hood) {
    return down(hood, DEFAULT_SPEED);
  }

  /** Move hood down at specified speed */
  public static Command down(Hood hood, double speed) {
    double output = -Math.abs(speed);
    return Commands.run(() -> hood.setOpenLoop(output), hood)
        .finallyDo(() -> hood.stop())
        .withName("HoodDown");
  }

  /** Manual control with joystick */
  public static Command manual(Hood hood, DoubleSupplier speed) {
    return Commands.run(() -> hood.setOpenLoop(speed.getAsDouble()), hood)
        .finallyDo(() -> hood.stop())
        .withName("HoodManual");
  }

  /** Stop hood */
  public static Command stop(Hood hood) {
    return Commands.runOnce(hood::stop, hood).withName("HoodStop");
  }
}
