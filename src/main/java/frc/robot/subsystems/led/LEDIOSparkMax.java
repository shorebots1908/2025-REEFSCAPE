package frc.robot.subsystems.led;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDIOSparkMax implements LEDIO {
  public Spark ledStrip = new Spark(0);

  private LEDConfig config;
  private LEDIO.LEDIOInputs inputs = new LEDIOInputs();

  public LEDIOSparkMax(NetworkTable FMS, LEDConfig config) {
    this.config = config;
    if (FMS.getEntry("isRedAlliance").getBoolean(false)) {
      inputs.teamColor = inputs.red;
      red();
    } else {
      inputs.teamColor = inputs.blue;
      blue();
    }
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

  public void yellow() {
    setLEDColor(inputs.yellow);
    inputs.color = "yellow";
  }

  public void green() {
    setLEDColor(inputs.green);
    inputs.color = "green";
  }

  public void blue() {
    setLEDColor(inputs.blue);
    inputs.color = "blue";
  }

  public void red() {
    setLEDColor(inputs.red);
    inputs.color = "red";
  }

  public void purple() {
    setLEDColor(inputs.purple);
    inputs.color = "purple";
  }

  public void teamColor() {
    setLEDColor(inputs.teamColor);
    if (inputs.teamColor == inputs.red) {
      inputs.color = "red";
    } else {
      inputs.color = "blue";
    }
  }
}
