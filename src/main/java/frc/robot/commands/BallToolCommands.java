package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ballerIntake.BallerIntake;

public class BallToolCommands {
  public static Command pickupBall(BallerIntake baller) {
    return Commands.run(
        () -> {
          //

        });
  }

  public static Command placeBall(BallerIntake baller) {
    return Commands.run(
        () -> {
          //

        });
  }
}
