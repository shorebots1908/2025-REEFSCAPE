package frc.robot.hybrid;

import java.util.ArrayList;

public class BlendedControl {
  private final ArrayList<ControlVectorProvider> pValues = new ArrayList<>();
  private final ArrayList<ControlVectorProvider> tValues = new ArrayList<>();

  /**
   * Add a new input component to the blended control by providing
   *
   * @param pValue function to get latest pValue for this component
   * @param tValue function to get latest tValue for this component
   */
  public void addComponent(ControlVectorProvider pValue, ControlVectorProvider tValue) {
    pValues.add(pValue);
    tValues.add(tValue);
  }

  /**
   * Multiply the components of all inputs by their corresponding weights, then sum them
   *
   * @return A SwerveVector representing the blend of all inputs
   */
  public ControlVector solve() {
    ControlVector output = new ControlVector();

    for (int i = 0; i < pValues.size(); i++) {
      ControlVector pX = pValues.get(i).get();
      ControlVector tX = tValues.get(i).get();

      ControlVector inputX = pX.times(tX);
      output = output.plus(inputX);
    }

    return output;
  }
}
