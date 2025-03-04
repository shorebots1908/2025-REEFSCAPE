package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ClimberCommands {

  public static final BasePosition DEPLOYED = new BasePosition(0.2);
  public static final BasePosition UNDEPLOYED = new BasePosition(0.0);

  public static Command climb(Climber climber) {
    return Commands.run(() -> {}, climber);
  }

  public static Command lower(Climber climber) {
    return Commands.run(() -> {}, climber);
  }

  // public static Command deployUndeploy(Climber climber) {
  //   return Commands.run(
  //       () -> {
  //         double position = climber.getPosition();
  //         if (climber.isDeployed()) {
  //           climber.setTargetPosition(position + (UNDEPLOYED.getValue() - DEPLOYED.getValue()));
  //         } else {
  //           climber.setTargetPosition(position + (DEPLOYED.getValue() - UNDEPLOYED.getValue()));
  //         }
  //         climber.toggleDeploy();
  //       },
  //       climber);
  // }

  public static Command joystick(Climber climber, DoubleSupplier output) {
    return Commands.run(
        () -> {
          climber.setOpenLoop(output.getAsDouble());
        },
        climber);
  }
}
