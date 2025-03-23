package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
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
      Logger.recordOutput("Intake/ActiveCommand", commandName);
    }
  }

  public void setFeedOpenLoop(double output) {
    io.setFeedOpenLoop(output);
  }

  public void feedStop() {
    io.feedStop();
  }

  public boolean isHolding() {
    return io.isHolding();
  }

  public boolean isEmpty() {
    return io.isEmpty();
  }
}
