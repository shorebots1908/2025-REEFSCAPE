package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommands {
  public static Command climb(Climber climber) {
    return Commands.run(() -> {}, climber);
  }

  public static Command lower(Climber climber) {
    return Commands.run(() -> {}, climber);
  }
}
