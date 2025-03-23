package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
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
      Logger.recordOutput("Wrist/ActiveCommand", commandName);
    }
  }

  public void setTargetPosition(BasePosition output) {
    io.setTargetPosition(output);
  }

  public void setWristOpenLoop(double output) {
    io.setWristOpenLoop(output);
  }

  public boolean atTargetPosition() {
    return io.atTargetPosition();
  }
}
