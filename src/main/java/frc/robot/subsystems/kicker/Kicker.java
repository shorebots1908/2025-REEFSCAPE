package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIO.FeederIOInputs inputs = new KickerIO.FeederIOInputs();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput(io.name() + "/Velocity", inputs.velocityRPS);
    Logger.recordOutput(io.name() + "/Current", inputs.currentAmps);
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
