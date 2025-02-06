package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralIntake.CoralIntake;

public class CoralToolCommands {
  public static Command coralPickup(CoralIntake intake) {
    return Commands.run(() -> {}, intake);
  }

  public static Command coralPlace(CoralIntake intake) {
    return Commands.run(() -> {}, intake);
  }
}
