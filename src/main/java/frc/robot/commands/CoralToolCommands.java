package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.coralTool.CoralTool;
import java.util.function.DoubleSupplier;

public class CoralToolCommands {
  public static final double FEED_SPEED = 5.0;

  public static Command moveByJoystick(
      CoralTool coralTool, DoubleSupplier wrist, DoubleSupplier wheels) {
    return Commands.run(
        () -> {
          coralTool.setWristOpenLoop(wrist.getAsDouble());
          coralTool.setFeedOpenLoop(wheels.getAsDouble());
        },
        coralTool);
  }

  public static Command pickup(CoralTool coralTool) {
    return Commands.sequence(feedOut(coralTool));
  }

  public static Command place(CoralTool coralTool) {
    return Commands.run(() -> {}, coralTool);
  }

  public static Command feedIn(CoralTool coralTool) {
    return Commands.run(
        () -> {
          coralTool.setFeedOpenLoop(-FEED_SPEED);
        },
        coralTool);
  }

  public static Command feedOut(CoralTool coralTool) {
    return Commands.run(
        () -> {
          coralTool.setFeedOpenLoop(FEED_SPEED);
        },
        coralTool);
  }

  public static Command feedStop(CoralTool coralTool) {
    return Commands.run(
        () -> {
          coralTool.feedStop();
        },
        coralTool);
  }

  public static Command setWristPosition(CoralTool coralTool, BasePosition position) {
    return Commands.runOnce(
        () -> {
          coralTool.setWristPosition(position);
        },
        coralTool);
  }

  public static Command isHolding(CoralTool coralTool) {
    return Commands.run(
        () -> {
          coralTool.isHolding();
        },
        coralTool);
  }

  public static Command waitUntilCoralIsHolding(CoralTool coralTool, boolean holding) {
    return Commands.idle(coralTool)
        .until(
            () -> {
              if (holding) {
                return coralTool.isHolding();
              } else {
                return !coralTool.isHolding();
              }
            })
        .withName("WaitUntilCoralIsHolding");
  }
}
