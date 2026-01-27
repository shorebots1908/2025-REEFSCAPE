package frc.robot.subsystems.kicker;

public class KickerIOSim implements KickerIO {
  private double output = 0.0;

  public KickerIOSim() {}

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.connected = true;
    inputs.velocityRPS = output * 100.0;
  }

  @Override
  public void setOpenLoop(double output) {
    this.output = output;
  }

  @Override
  public void stop() {
    output = 0.0;
  }
}
