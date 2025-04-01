package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BasePosition;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private TrapezoidProfile profile;
  private ElevatorFeedforward feedforward;
  private Timer profileTimer;
  private State t0State;
  private BasePosition previousTarget;

  public Elevator(ElevatorIO io) {
    this.io = io;

    profile = new TrapezoidProfile(new Constraints(800, 1200));
    feedforward = new ElevatorFeedforward(0.10, 0.10, 3.07);
    profileTimer = new Timer();
    profileTimer.start();
    previousTarget = new BasePosition(0);

    t0State = new State(this.io.getEncoder().getPosition(), this.io.getEncoder().getVelocity());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    io.periodic();
    var config = io.getConfig();
    var outputState =
        profile.calculate(
            Timer.getTimestamp(),
            t0State,
            new State(
                previousTarget.toRange(config.encoderLowerLimit, config.encoderUpperLimit), 0));
    var outputPosition = outputState.position;

    io.setTargetPosition(
        BasePosition.fromRange(config.encoderLowerLimit, config.encoderUpperLimit, outputPosition));
  }

  public void setTargetPosition(BasePosition position) {
    if (previousTarget.getValue() != position.getValue()) {
      profileTimer.reset();
      t0State = new State(this.io.getEncoder().getPosition(), this.io.getEncoder().getVelocity());
    }

    previousTarget = position;
    profileTimer.start();
    // io.setTargetPosition(position);
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
