package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  private final CoralIntakeIO io;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }
}
