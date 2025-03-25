package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ClimberCommands {
  public static final BasePosition DEPLOYED = new BasePosition(0.2);
  public static final BasePosition UNDEPLOYED = new BasePosition(0.0);

  public static Command joystick(Climber climber, DoubleSupplier output) {
    return Commands.run(
        () -> {
          climber.setOpenLoop(output.getAsDouble());
        },
        climber);
  }
}
