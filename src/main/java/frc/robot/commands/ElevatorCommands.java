package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

  public static Command elevatorMove(Elevator elevator, BasePosition position) {
    return Commands.run(
        () -> {
          //

        });
  }
}
