package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;
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
  }

  public void setTargetPosition(BasePosition output) {
    io.setTargetPosition(output);
  }

  public void setWristOpenLoop(double output) {
    io.setWristOpenLoop(output);
  }

  public void setFeedOpenLoop(double output) {
    io.setFeedOpenLoop(output);
  }

  public void feedStop() {
    io.feedStop();
  }

  public boolean isHolding() {
    return false;
  }

  public boolean isEmpty() {
    return false;
  }

  public boolean atTargetPosition() {
    return io.atTargetPosition();
  }
}
