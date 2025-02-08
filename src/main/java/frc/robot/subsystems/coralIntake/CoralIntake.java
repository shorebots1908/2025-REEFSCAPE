package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;

public class CoralIntake extends SubsystemBase {
  private final CoralIntakeIO io;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }

  public void goToPosition(BasePosition position) {}

  public void positionStop() {}

  public void spinIn(double speed) {}

  public void spinOut(double speed) {}

  public void spinStop() {}
}
