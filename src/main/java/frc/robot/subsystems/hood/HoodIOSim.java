package frc.robot.subsystems.hood;

public class HoodIOSim implements HoodIO {
  private double position = 0.2; // Start at 10%

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = position;
  }

  @Override
  public void setPosition(double position) {
    this.position = Math.max(0.0, Math.min(1.0, position));
  }
}
