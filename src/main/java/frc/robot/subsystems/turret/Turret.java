package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(io.name(), inputs);
    io.periodic();

    Command active = this.getCurrentCommand();
    if (active != null) {
      Logger.recordOutput(io.name() + "/ActiveCommand", active.getName());
    }
  }

  // Turn motor methods
  public void setTurnPosition(double positionRotations) {
    io.setTurnPosition(positionRotations);
  }

  public void setTurnOpenLoop(double output) {
    io.setTurnOpenLoop(output);
  }

  public void turnStop() {
    io.turnStop();
  }

  public double getTurnPosition() {
    return io.getTurnPosition();
  }

  public boolean isAtTurnTarget(double targetRotations, double toleranceRotations) {
    return io.isAtTurnTarget(targetRotations, toleranceRotations);
  }

  // Shoot motor methods
  public void setShootVelocity(double velocityRPS) {
    io.setShootVelocity(velocityRPS);
  }

  public void setShootOpenLoop(double output) {
    io.setShootOpenLoop(output);
  }

  public void shootStop() {
    io.shootStop();
  }

  public double getShootVelocity() {
    return io.getShootVelocity();
  }
}
