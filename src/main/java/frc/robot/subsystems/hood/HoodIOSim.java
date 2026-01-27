package frc.robot.subsystems.hood;

public class HoodIOSim implements HoodIO {
  private double output = 0.0;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedOutput = output;
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
