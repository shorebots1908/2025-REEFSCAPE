package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIO.SpindexerIOInputs inputs = new SpindexerIO.SpindexerIOInputs();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput(io.name() + "/Velocity", inputs.velocityRPM);
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
