package frc.robot.subsystems.spindexer;

public interface SpindexerIO {
  public static class SpindexerIOInputs {
    public boolean connected = false;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void periodic() {}

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default String name() {
    return "Spindexer";
  }

  public default void setOpenLoop(double output) {}

  public default void stop() {}
}
