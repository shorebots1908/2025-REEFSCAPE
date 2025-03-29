package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  public LED(LEDIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(io.name(), inputs);
    io.periodic();

    Command active = this.getCurrentCommand();
    if (active != null) {
      String commandName = active.getName();
      Logger.recordOutput("LED/ActiveCommand", commandName);
    }
  }

  public void setLEDColor(double pwmColorCode) {}

  public void currentLEDColor() {}

  public void yellow() {}

  public void green() {}

  public void blue() {}

  public void red() {}

  public void purple() {}

  public void teamColor() {}
}
