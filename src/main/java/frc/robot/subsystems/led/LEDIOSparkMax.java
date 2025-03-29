package frc.robot.subsystems.led;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDIOSparkMax implements LEDIO {

  private LEDConfig config;
  private LEDIO.LEDIOInputs inputs = new LEDIOInputs();
  public Spark ledStrip = new Spark(0);

  public LEDIOSparkMax(NetworkTable FMS, LEDConfig config) {
    this.config = config;
    if (FMS.getEntry("IsRedAlliance").getBoolean(false)) {
      inputs.teamColor = inputs.red;

    } else {
      inputs.teamColor = inputs.blue;
    }
    teamColor();
  }

  @Override
  public void updateInputs(LEDIO.LEDIOInputs inputs) {
    this.inputs = inputs;
  }

  public void setLEDColor(double pwmColorCode) {
    ledStrip.set(pwmColorCode);
  }

  public void currentLEDColor() {
    ledStrip.get();
  }

  public void periodic() {
    SmartDashboard.putString("Current LED Color", inputs.color);
  }

  @Override
  public void yellow() {
    setLEDColor(inputs.yellow);
    inputs.color = "yellow";
  }

  @Override
  public void green() {
    setLEDColor(inputs.green);
    inputs.color = "green";
  }

  @Override
  public void blue() {
    setLEDColor(inputs.blue);
    inputs.color = "blue";
  }

  @Override
  public void red() {
    setLEDColor(inputs.red);
    inputs.color = "red";
  }

  @Override
  public void purple() {
    setLEDColor(inputs.purple);
    inputs.color = "purple";
  }

  @Override
  public void teamColor() {
    setLEDColor(inputs.teamColor);
    if (inputs.teamColor == inputs.red) {
      inputs.color = "red";
    } else {
      inputs.color = "blue";
    }
  }
}
