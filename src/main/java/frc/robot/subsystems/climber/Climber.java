package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;

public class Climber extends SubsystemBase {
  private ClimberIO io;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void goToPosition(BasePosition position) {}

  public void positionStop() {}
}
