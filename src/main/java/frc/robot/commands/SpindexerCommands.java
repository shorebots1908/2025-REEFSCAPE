package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.spindexer.Spindexer;
import java.util.function.DoubleSupplier;

public class SpindexerCommands {
  public static final double DEFAULT_SPEED = 0.4; // 40% speed

  /** Run spindexer forward at default speed */
  public static Command forward(Spindexer spindexer) {
    return forward(spindexer, DEFAULT_SPEED);
  }

  /** Run spindexer forward at specified speed */
  public static Command forward(Spindexer spindexer, double speed) {
    double output = Math.abs(speed);
    return Commands.run(() -> spindexer.setOpenLoop(output), spindexer)
        .finallyDo(() -> spindexer.stop())
        .withName("SpindexerForward");
  }

  /** Run spindexer reverse at default speed */
  public static Command reverse(Spindexer spindexer) {
    return reverse(spindexer, DEFAULT_SPEED);
  }

  /** Run spindexer reverse at specified speed */
  public static Command reverse(Spindexer spindexer, double speed) {
    double output = -Math.abs(speed);
    return Commands.run(() -> spindexer.setOpenLoop(output), spindexer)
        .finallyDo(() -> spindexer.stop())
        .withName("SpindexerReverse");
  }

  /** Manual control with joystick */
  public static Command manual(Spindexer spindexer, DoubleSupplier speed) {
    return Commands.run(() -> spindexer.setOpenLoop(speed.getAsDouble()), spindexer)
        .finallyDo(() -> spindexer.stop())
        .withName("SpindexerManual");
  }

  /** Stop spindexer */
  public static Command stop(Spindexer spindexer) {
    return Commands.runOnce(spindexer::stop, spindexer).withName("SpindexerStop");
  }
}
