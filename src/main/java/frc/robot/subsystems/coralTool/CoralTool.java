package frc.robot.subsystems.coralTool;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;

public class CoralTool extends SubsystemBase {
  private final CoralToolIO io;

  public CoralTool(CoralToolIO io) {
    this.io = io;
  }

  public void goToPosition(BasePosition position) {}

  public void setWristOpenLoop(double output) {
    io.setWristOpenLoop(output);
  }

  public void setWristPosition(BasePosition output) {
    io.setWristPosition(output);
  }

  public void setFeedOpenLoop(double output) {
    io.setFeedOpenLoop(output);
  }

  public void positionStop() {}

  public void spinIn(double speed) {}

  public void spinOut(double speed) {}

  public void feedStop() {
    io.feedStop();
  }

  public boolean isHolding() {

    return false;
  }

  public boolean isEmpty() {
    return false;
  }
}
