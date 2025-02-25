package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorCommands {
  public static final BasePosition STATION = new BasePosition(0);
  public static final BasePosition L2 = new BasePosition(0);
  public static final BasePosition L3 = new BasePosition(0);
  public static final BasePosition L4 = new BasePosition(0);

  public static Command moveByJoystick(Elevator elevator, DoubleSupplier value) {
    return Commands.run(
        () -> {
          elevator.setElevatorOpenLoop(value.getAsDouble());
        },
        elevator);
  }

  public static Command setTargetPosition(Elevator elevator, BasePosition position) {
    return Commands.runOnce(
        () -> {
          elevator.setTargetPosition(position);
        },
        elevator);
  }

  public static Command waitUntilElevatorAtTargetPosition(Elevator elevator) {
    return Commands.idle(elevator)
        .until(elevator::atTargetPosition)
        .withName("WaitUntilElevatorAtTargetPosition");
  }

  public static Command moveElevatorTunable(Elevator elevator, LoggedNetworkNumber number) {
    return Commands.run(
        () -> {
          elevator.setTargetPosition(new BasePosition(number.get()));
        },
        elevator);
  }

  // Auto Commands
  public static Command upAndDown(Elevator elevator) {
    return Commands.sequence(
        setTargetPosition(elevator, new BasePosition(1)),
        waitUntilElevatorAtTargetPosition(elevator).withTimeout(1),
        setTargetPosition(elevator, new BasePosition(0)),
        waitUntilElevatorAtTargetPosition(elevator).withTimeout(1));
  }

  public static Command goToPosition(Elevator elevator, BasePosition position) {
    return Commands.sequence(
        setTargetPosition(elevator, position),
        waitUntilElevatorAtTargetPosition(elevator).withTimeout(1));
  }
}
