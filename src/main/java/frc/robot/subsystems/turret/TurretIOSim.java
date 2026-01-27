package frc.robot.subsystems.turret;

public class TurretIOSim implements TurretIO {
  private double turnPosition = 0.0;
  private double turnOutput = 0.0;
  private double shootVelocity = 0.0;
  private double shootOutput = 0.0;

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turnConnected = true;
    inputs.shootConnected = true;
    inputs.turnPositionRotations = turnPosition;
    inputs.turnVelocityRPS = turnOutput * 10.0;
    inputs.shootVelocityRPS = shootVelocity;
  }

  @Override
  public void setTurnPosition(double positionRotations) {
    double error = positionRotations - turnPosition;
    turnPosition += error * 0.1;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnOutput = output;
    turnPosition += output * 0.02;
  }

  @Override
  public void turnStop() {
    turnOutput = 0.0;
  }

  @Override
  public double getTurnPosition() {
    return turnPosition;
  }

  @Override
  public void setShootVelocity(double velocityRPS) {
    double error = velocityRPS - shootVelocity;
    shootVelocity += error * 0.1;
  }

  @Override
  public void setShootOpenLoop(double output) {
    shootOutput = output;
    shootVelocity = output * 100.0;
  }

  @Override
  public void shootStop() {
    shootOutput = 0.0;
    shootVelocity = 0.0;
  }

  @Override
  public double getShootVelocity() {
    return shootVelocity;
  }
}
