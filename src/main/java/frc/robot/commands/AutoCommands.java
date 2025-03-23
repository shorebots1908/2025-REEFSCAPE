package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class AutoCommands {
  public static Command score(Intake coral, Wrist wrist, Elevator elevator, BasePosition position) {
    return Commands.sequence(
        WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_DOWN).withTimeout(0.5),
        ElevatorCommands.setTargetPosition(elevator, position).withTimeout(0.1),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator).withTimeout(2),
        WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_SCORE).withTimeout(0.5),
        IntakeCommands.feedOut(coral).withTimeout(0.5),
        IntakeCommands.waitUntilCoralIsHolding(coral, false)
        // IntakeCommands.feedStop(intake)
        );
  }

  public static Command scoreL4(
      Intake coral, Wrist wrist, Elevator elevator, BasePosition position) {
    return Commands.sequence(
        WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_DOWN).withTimeout(1.0),
        ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.CORAL_L4).withTimeout(1.0),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator).withTimeout(2),
        WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_SCORE).withTimeout(0.5),
        IntakeCommands.feedOut(coral).withTimeout(0.5),
        IntakeCommands.waitUntilCoralIsHolding(coral, false)
        // IntakeCommands.feedStop(intake)
        );
  }

  public static Command smartElevator(
      Elevator elevator, Wrist wrist, BasePosition elevatorPosition) {
    return Commands.sequence(
        Commands.parallel(
            ElevatorCommands.goToPosition(elevator, elevatorPosition),
            WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_DOWN)),
        WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_SCORE));
  }

  public static Command smartElevatordown(
      Elevator elevator, Wrist wrist, BasePosition elevatorPosition) {
    return Commands.sequence(
        ElevatorCommands.goToPosition(elevator, elevatorPosition),
        WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_INTAKE));
  }

  public static Command smartElevatorl3(
      Elevator elevator, Wrist wrist, BasePosition elevatorPosition) {
    return Commands.sequence(
        Commands.parallel(
            ElevatorCommands.goToPosition(elevator, elevatorPosition),
            WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_DOWN)),
        WristCommands.goToPosition(wrist, WristCommands.CORAL_WRIST_L3));
  }
}
