package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class AutoCommands {
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
