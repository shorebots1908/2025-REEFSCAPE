package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  public static final double FEED_SPEED = 5.0;

  public static Command moveByJoystick(
      Intake coralTool, DoubleSupplier wrist, DoubleSupplier wheels) {
    return Commands.run(
        () -> {
          coralTool.setWristOpenLoop(wrist.getAsDouble());
          coralTool.setFeedOpenLoop(wheels.getAsDouble());
        },
        coralTool);
  }

  public static Command pickup(Intake coralTool) {
    return Commands.sequence(feedOut(coralTool));
  }

  public static Command place(Intake coralTool) {
    return Commands.run(() -> {}, coralTool);
  }

  public static Command feedIn(Intake coralTool) {
    return Commands.run(
        () -> {
          coralTool.setFeedOpenLoop(-FEED_SPEED);
        },
        coralTool);
  }

  public static Command feedOut(Intake coralTool) {
    return Commands.run(
        () -> {
          coralTool.setFeedOpenLoop(FEED_SPEED);
        },
        coralTool);
  }

  public static Command setTargetPosition(Intake coralTool, BasePosition position) {
    return Commands.runOnce(
        () -> {
          coralTool.setTargetPosition(position);
        },
        coralTool);
  }

  public static Command isHolding(Intake coralTool) {
    return Commands.run(
        () -> {
          coralTool.isHolding();
        },
        coralTool);
  }

  public static Command waitUntilCoralAtTargetPosition(Intake coralTool) {
    return Commands.idle(coralTool)
        .until(coralTool::atTargetPosition)
        .withName("WaitUntilCoralAtTargetPosition");
  }

  public static Command waitUntilCoralIsHolding(Intake coralTool, boolean holding) {
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

  public static Command waitUntilElevatorAtTargetPosition(Intake coralTool) {
    return Commands.idle(coralTool)
        .until(coralTool::atTargetPosition)
        .withName("WaitUntilElevatorAtTargetPosition");
  }

  public static Command goToPosition(Intake coralTool, BasePosition position) {
    return Commands.sequence(
        setTargetPosition(coralTool, position),
        waitUntilCoralAtTargetPosition(coralTool).withTimeout(1));
  }
}
