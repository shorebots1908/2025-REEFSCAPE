package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralTool.CoralTool;

public class CoralToolCommands {
  public static Command moveByJoystick(CoralTool coralTool, DoubleSupplier output) {
    return Commands.run(() -> {
      coralTool.setWristOpenLoop(output.getAsDouble());
    }, coralTool);
  }

  public static Command coralPickup(CoralTool coralTool) {
    return Commands.run(() -> {}, coralTool);
  }

  public static Command coralPlace(CoralTool coralTool) {
    return Commands.run(() -> {}, coralTool);
  }
}
