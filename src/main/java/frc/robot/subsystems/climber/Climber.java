package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    io.periodic();
  }

  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  public void setTargetPosition(BasePosition position) {
    io.setTargetPosition(position);
  }

  public BasePosition getTargetPosition() {
    return io.getTargetPosition();
  }

  public boolean atTargetPosition() {
    // return io.atTargetPosition();
    return false; // TODO
  }

  public void positionStop() {
    io.positionStop();
  }

  public Rotation2d getRotation() {
    return io.getRotation();
  }
}
