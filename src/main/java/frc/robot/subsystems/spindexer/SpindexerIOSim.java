package frc.robot.subsystems.spindexer;

public class SpindexerIOSim implements SpindexerIO {
  private double output = 0.0;

  public SpindexerIOSim() {}

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.connected = true;
    inputs.velocityRPM = output * 5700.0; // NEO free speed
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
