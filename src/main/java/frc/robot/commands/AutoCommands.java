package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class AutoCommands {
  public static Command pickup(Intake coral, Elevator elevator) {
    return Commands.sequence(
        ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.BOTTOM),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator),
        IntakeCommands.pickup(coral),
        IntakeCommands.waitUntilCoralIsHolding(coral, true)
        // IntakeCommands.feedStop(intake)
        );
  }

  public static Command score(Intake coral, Elevator elevator, BasePosition position) {
    return Commands.sequence(
        IntakeCommands.goToPosition(coral, IntakeCommands.CORAL_WRIST_DOWN).withTimeout(0.5),
        ElevatorCommands.setTargetPosition(elevator, position).withTimeout(0.1),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator).withTimeout(2),
        IntakeCommands.goToPosition(coral, IntakeCommands.CORAL_WRIST_SCORE).withTimeout(0.5),
        IntakeCommands.feedOut(coral).withTimeout(0.5),
        IntakeCommands.waitUntilCoralIsHolding(coral, false)
        // IntakeCommands.feedStop(intake)
        );
  }

  public static Command scoreL4(Intake coral, Elevator elevator, BasePosition position) {
    return Commands.sequence(
        IntakeCommands.goToPosition(coral, IntakeCommands.CORAL_WRIST_DOWN).withTimeout(1.0),
        ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.CORAL_L4).withTimeout(1.0),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator).withTimeout(2),
        IntakeCommands.goToPosition(coral, IntakeCommands.CORAL_WRIST_SCORE).withTimeout(0.5),
        IntakeCommands.feedOut(coral).withTimeout(0.5),
        IntakeCommands.waitUntilCoralIsHolding(coral, false)
        // IntakeCommands.feedStop(intake)
        );
  }

  public static Command smartElevator(
      Elevator elevator, Intake coral, BasePosition elevatorPosition) {
    return Commands.sequence(
        Commands.parallel(
            ElevatorCommands.goToPosition(elevator, elevatorPosition),
            IntakeCommands.goToPosition(coral, IntakeCommands.CORAL_WRIST_DOWN)),
        IntakeCommands.goToPosition(coral, IntakeCommands.CORAL_WRIST_SCORE));
  }

  public static Command smartElevatordown(
      Elevator elevator, Intake coral, BasePosition elevatorPosition) {
    return Commands.sequence(
        ElevatorCommands.goToPosition(elevator, elevatorPosition),
        IntakeCommands.goToPosition(coral, IntakeCommands.CORAL_WRIST_INTAKE));
  }
}
