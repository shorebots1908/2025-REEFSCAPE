package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {
  public static final BasePosition BOTTOM = new BasePosition(0.0); // 0
  public static final BasePosition CORAL_L2 = new BasePosition(0.066); // 4.515
  public static final BasePosition CORAL_L3 = new BasePosition(0.35); // 29.025 ---- was 0.427
  public static final BasePosition CORAL_L4 = new BasePosition(1.0); // 64.5
  public static final BasePosition ALGAE_L2 = new BasePosition(0.569); // 38.7
  public static final BasePosition ALGAE_L3 = new BasePosition(0.873); // 59.34
  public static final BasePosition ALGAE_PROCESSOR = new BasePosition(0.218); // 14.835

  public static Command moveByJoystick(Elevator elevator, DoubleSupplier value) {
    return Commands.run(
            () -> {
              elevator.setElevatorOpenLoop(value.getAsDouble());
            },
            elevator)
        .finallyDo(() -> {});
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

  public static Command goToPosition(Elevator elevator, BasePosition position) {
    return Commands.sequence(
        setTargetPosition(elevator, position),
        waitUntilElevatorAtTargetPosition(elevator).withTimeout(1));
  }
}
