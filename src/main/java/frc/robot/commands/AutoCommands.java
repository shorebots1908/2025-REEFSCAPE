package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.coralTool.CoralTool;
import frc.robot.subsystems.elevator.Elevator;

public class AutoCommands {
  public CoralToolCommands coralToolCommands;
  public CoralTool coralTool;

  public static Command pickup(CoralTool coralTool, Elevator elevator) {

    return Commands.sequence(
        ElevatorCommands.setTargetPosition(elevator, new BasePosition(0.5)),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator),
        CoralToolCommands.pickup(coralTool),
        CoralToolCommands.waitUntilCoralIsHolding(coralTool, true),
        CoralToolCommands.feedStop(coralTool));
  }

  public static Command score(CoralTool coralTool, Elevator elevator, BasePosition position) {

    return Commands.sequence(
        ElevatorCommands.setTargetPosition(elevator, position),
        ElevatorCommands.waitUntilElevatorAtTargetPosition(elevator),
        CoralToolCommands.place(coralTool),
        CoralToolCommands.waitUntilCoralIsHolding(coralTool, false),
        CoralToolCommands.feedStop(coralTool));
  }

  // public static Command (CoralTool coralTool, Elevator elevator) {

  //     return Commands.sequence(
  //         ElevatorCommands.setTargetPosition(elevator, new BasePosition(0.5)),
  //         Commands.idle(elevator).until(() -> elevator.atTargetPosition()),
  //         CoralToolCommands.pickup(coralTool),
  //         Commands.idle(coralTool).until(() -> coralTool.isHolding()),
  //         CoralToolCommands.feedStop(coralTool)

  //     );
  // }

}
