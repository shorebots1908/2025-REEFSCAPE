package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class AutoCommands {
  public static Command pickup(Intake intake, Elevator elevator) {
    return Commands.sequence(
        ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.BOTTOM),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator),
        IntakeCommands.pickup(intake),
        IntakeCommands.waitUntilCoralIsHolding(intake, true)
        // IntakeCommands.feedStop(intake)
        );
  }

  public static Command score(Intake intake, Elevator elevator, BasePosition position) {
    return Commands.sequence(
        ElevatorCommands.setTargetPosition(elevator, position),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator),
        IntakeCommands.place(intake),
        IntakeCommands.waitUntilCoralIsHolding(intake, false)
        // IntakeCommands.feedStop(intake)
        );
  }
}
