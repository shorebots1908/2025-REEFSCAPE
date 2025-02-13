package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralTool.CoralTool;
import java.util.function.DoubleSupplier;

public class CoralToolCommands {
  public static Command moveByJoystick(CoralTool coralTool, DoubleSupplier wrist, DoubleSupplier wheels) {
    return Commands.run(
        () -> {
          coralTool.setWristOpenLoop(wrist.getAsDouble());
          coralTool.setWheelsOpenLoop(wheels.getAsDouble());
        },
        coralTool);
  }

  public static Command coralPickup(CoralTool coralTool) {
    return Commands.run(() -> {}, coralTool);
  }

  public static Command coralPlace(CoralTool coralTool) {
    return Commands.run(() -> {}, coralTool);
  }
}
