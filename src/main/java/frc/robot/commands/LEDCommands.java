package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;

public class LEDCommands {

  public static Command ledDefault(LED led, Intake intake) {
    return Commands.run(
        () -> {
          if (intake.isHolding()) {
            // If the sensor has a coral, keep feeding in to hold it
            led.yellow();
          } else {
            led.teamColor();
          }
        },
        led);
  }
  public static Command ledChange(LED led) {
    return Commands.run(
        () -> {
        led.setLEDColor(0.67);}
        , led);
  }
}
