package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class AutoCommands {
  public static Command pickup(Intake coralTool, Elevator elevator) {
    return Commands.sequence(
        ElevatorCommands.setTargetPosition(elevator, new BasePosition(0.5)),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator),
        IntakeCommands.pickup(coralTool),
        IntakeCommands.waitUntilCoralIsHolding(coralTool, true)
        // IntakeCommands.feedStop(coralTool)
        );
  }

  public static Command score(Intake coralTool, Elevator elevator, BasePosition position) {
    return Commands.sequence(
        ElevatorCommands.setTargetPosition(elevator, position),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator),
        IntakeCommands.place(coralTool),
        IntakeCommands.waitUntilCoralIsHolding(coralTool, false)
        // IntakeCommands.feedStop(coralTool)
        );
  }
}
