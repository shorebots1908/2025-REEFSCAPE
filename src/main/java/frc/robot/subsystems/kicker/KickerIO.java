package frc.robot.subsystems.kicker;

public interface KickerIO {
  public static class FeederIOInputs {
    public boolean connected = false;
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void periodic() {}

  public default void updateInputs(FeederIOInputs inputs) {}

  public default String name() {
    return "Feeder";
  }

  public default void setOpenLoop(double output) {}

  public default void stop() {}
}
