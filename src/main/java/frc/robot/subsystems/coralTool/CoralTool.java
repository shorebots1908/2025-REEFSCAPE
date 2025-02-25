package frc.robot.subsystems.coralTool;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;

public class CoralTool extends SubsystemBase {
  private final CoralToolIO io;

  public CoralTool(CoralToolIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
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
