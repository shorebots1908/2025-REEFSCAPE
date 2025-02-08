package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {

  public static Command moveByJoystick(Elevator elevator, DoubleSupplier value) {
    return Commands.run(
        () -> {
          elevator.setElevatorOpenLoop(value.getAsDouble());
        },
        elevator);
  }

  public static Command moveToPosition(Elevator elevator, BasePosition position) {
    return Commands.run(
        () -> {
          elevator.setTargetPosition(position);
        },
        elevator);
  }

  public static Command stop(Elevator elevator) {
    return Commands.run(
        () -> {
          elevator.stop();
        });
  }
}
