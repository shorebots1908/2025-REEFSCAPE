package frc.robot.subsystems.elevator;

public class ElevatorPosition {
  /** Must be in the range of 0.0 to 1.0 */
  private double value;

  public ElevatorPosition(double value) {
    if (value < 0.0) {
      throw new RuntimeException("ElevatorPosition cannot be less than 0.0");
    }
    if (value > 1.0) {
      throw new RuntimeException("ElevatorPosition cannot be more than 1.0");
    }
    this.value = value;
  }

  public double getValue() {
    return this.value;
  }

  /**
   * Scales the inner range of 0.0 to 1.0 to the given range.
   *
   * <p>Test:
   *
   * <p>y1 = lower = -2.0 y2 = upper = 2.0
   *
   * <p>x -> y 0.0 -> 0.0 0.5 -> 0.63 1.0 -> 0.75
   *
   * <p>Correct x -> y 0.0 -> -2.0
   */
  public double toRange(double lower, double upper) {
    double x1 = 0.0;
    double x2 = 1.0;
    double y1 = lower;
    double y2 = upper;
    double m = (y2 - y1) / (x2 - x1);

    // y = mx + b
    // y - mx = b
    // b = y - mx
    // b = y2 - m(x2)
    double b = y2 - m * x2;

    // y = mx + b
    double x = this.value;
    double y = m * x + b;

    return y;
  }
}
