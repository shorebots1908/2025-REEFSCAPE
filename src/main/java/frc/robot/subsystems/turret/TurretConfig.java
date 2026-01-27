package frc.robot.subsystems.turret;

public class TurretConfig {
  public final String name;
  public final int turnMotorId;
  public final int shootMotorId;
  public final double turnPGain;
  public final double turnIGain;
  public final double turnDGain;
  public final boolean turnMotorInvert;
  public final boolean shootMotorInvert;
  public final double minRotations;
  public final double maxRotations;
  public final double homeRotations;

  public TurretConfig(
      String name,
      int turnMotorId,
      int shootMotorId,
      double turnP,
      double turnI,
      double turnD,
      boolean turnMotorInvert,
      boolean shootMotorInvert,
      double minRotations,
      double maxRotations,
      double homeRotations) {
    this.name = name;
    this.turnMotorId = turnMotorId;
    this.shootMotorId = shootMotorId;
    this.turnPGain = turnP;
    this.turnIGain = turnI;
    this.turnDGain = turnD;
    this.turnMotorInvert = turnMotorInvert;
    this.shootMotorInvert = shootMotorInvert;
    this.minRotations = minRotations;
    this.maxRotations = maxRotations;
    this.homeRotations = homeRotations;
  }
}
