package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput(io.name() + "/AppliedOutput", inputs.appliedOutput);
    io.periodic();

    Command active = this.getCurrentCommand();
    if (active != null) {
      Logger.recordOutput(io.name() + "/ActiveCommand", active.getName());
    }
  }

  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  public void stop() {
    io.stop();
  }
}
