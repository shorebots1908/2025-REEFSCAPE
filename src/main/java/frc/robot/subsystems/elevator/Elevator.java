package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    io.periodic();
  }

  public void setTargetPosition(BasePosition position) {
    io.setTargetPosition(position);
  }

  public boolean atTargetPosition() {
    return io.atTargetPosition();
  }

  public void setElevatorOpenLoop(double output) {
    io.setElevatorOpenLoop(output);
  }

  public void stop() {
    io.stop();
  }

  public boolean limitUpper() {
    return inputs.atUpper;
  }

  public boolean limitLower() {
    return inputs.atLower;
  }

  public double getPosition() {
    return inputs.positionRad;
  }
}
