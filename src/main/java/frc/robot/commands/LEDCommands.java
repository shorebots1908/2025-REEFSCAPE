package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.led.LED;

public class LEDCommands {

  public static Command ledDefault(LED led) {
    return Commands.run(
        () -> {
          var alliance = DriverStation.getAlliance();

          if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
              led.blue();
            }

            if (alliance.get() == Alliance.Red) {
              led.red();
            }
          }
        },
        led);
  }

  public static Command ledChange(LED led) {
    return Commands.run(
        () -> {
          led.setLEDColor(0.69);
        },
        led);
  }

  public static Command ledAlliance(LED led) {
    return Commands.run(
        () -> {
          led.updateTeamColor();
        },
        led);
  }
}
