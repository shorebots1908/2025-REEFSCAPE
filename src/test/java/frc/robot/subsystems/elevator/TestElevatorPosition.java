package frc.robot.subsystems.elevator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.BasePosition;
import org.junit.jupiter.api.Test;

public class TestElevatorPosition {
  @Test
  public void test() {
    BasePosition position = new BasePosition(0.5);
    double output = position.toRange(-2.0, 2.0); // Must be 0.0

    assertEquals(output, 0.0);
  }

  @Test
  public void test2() {
    BasePosition position = new BasePosition(0.5);
    double output = position.toRange(0.0, 10.0);

    assertEquals(output, 5.0);
  }
}
