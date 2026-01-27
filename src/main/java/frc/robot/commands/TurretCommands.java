package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.turret.Turret;
import java.util.function.DoubleSupplier;

public class TurretCommands {
  public static final double TURN_SPEED = 0.05;
  public static final double SHOOT_SPEED = 0.7;
  public static final double TURN_TOLERANCE = 0.5; // rotations

  // Turn position constants (in motor rotations)
  public static final double TURRET_HOME = 0.0;
  public static final double TURRET_LEFT = -5.0;
  public static final double TURRET_RIGHT = 5.0;

  // Shoot velocity constants (rotations per second)
  public static final double SHOOT_VELOCITY_LOW = 30.0;
  public static final double SHOOT_VELOCITY_HIGH = 80.0;

  // ==================== TURN COMMANDS ====================

  /** Turn turret to a specific position using closed-loop control */
  public static Command turnToPosition(Turret turret, double targetRotations) {
    return Commands.run(() -> turret.setTurnPosition(targetRotations), turret)
        .until(() -> turret.isAtTurnTarget(targetRotations, TURN_TOLERANCE))
        .finallyDo(() -> turret.turnStop())
        .withName("TurnToPosition");
  }

  /** Turn turret to home position */
  public static Command turnHome(Turret turret) {
    return turnToPosition(turret, TURRET_HOME).withName("TurnHome");
  }

  /** Turn turret to left position */
  public static Command turnLeft(Turret turret) {
    return turnToPosition(turret, TURRET_LEFT).withName("TurnLeft");
  }

  /** Turn turret to right position */
  public static Command turnRight(Turret turret) {
    return turnToPosition(turret, TURRET_RIGHT).withName("TurnRight");
  }

  /** Turn turret in positive direction at fixed speed */
  public static Command turnPositive(Turret turret) {
    return turnPositive(turret, TURN_SPEED);
  }

  /** Turn turret in positive direction at specified speed */
  public static Command turnPositive(Turret turret, double speed) {
    double output = Math.abs(speed);
    return Commands.run(() -> turret.setTurnOpenLoop(output), turret)
        .finallyDo(() -> turret.turnStop())
        .withName("TurnPositive");
  }

  /** Turn turret in negative direction at fixed speed */
  public static Command turnNegative(Turret turret) {
    return turnNegative(turret, TURN_SPEED);
  }

  /** Turn turret in negative direction at specified speed */
  public static Command turnNegative(Turret turret, double speed) {
    double output = -Math.abs(speed);
    return Commands.run(() -> turret.setTurnOpenLoop(output), turret)
        .finallyDo(() -> turret.turnStop())
        .withName("TurnNegative");
  }

  /** Manual control of turret turning with joystick */
  public static Command turnManual(Turret turret, DoubleSupplier speed) {
    return Commands.run(() -> turret.setTurnOpenLoop(speed.getAsDouble()), turret)
        .finallyDo(() -> turret.turnStop())
        .withName("TurnManual");
  }

  // ==================== SHOOT COMMANDS ====================

  /** Run shooter at specified velocity (rotations per second) */
  public static Command shootAtVelocity(Turret turret, double velocityRPS) {
    return Commands.run(() -> turret.setShootVelocity(velocityRPS), turret)
        .finallyDo(() -> turret.shootStop())
        .withName("ShootAtVelocity");
  }

  /** Run shooter at low velocity */
  public static Command shootLow(Turret turret) {
    return shootAtVelocity(turret, SHOOT_VELOCITY_LOW).withName("ShootLow");
  }

  /** Run shooter at high velocity */
  public static Command shootHigh(Turret turret) {
    return shootAtVelocity(turret, SHOOT_VELOCITY_HIGH).withName("ShootHigh");
  }

  /** Run shooter in positive direction at fixed speed (open loop) */
  public static Command shootPositive(Turret turret) {
    return shootPositive(turret, SHOOT_SPEED);
  }

  /** Run shooter in positive direction at specified speed (open loop) */
  public static Command shootPositive(Turret turret, double speed) {
    double output = Math.abs(speed);
    return Commands.run(() -> turret.setShootOpenLoop(output), turret)
        .finallyDo(() -> turret.shootStop())
        .withName("ShootPositive");
  }

  /** Run shooter in negative direction at fixed speed (open loop) */
  public static Command shootNegative(Turret turret) {
    return shootNegative(turret, SHOOT_SPEED);
  }

  /** Run shooter in negative direction at specified speed (open loop) */
  public static Command shootNegative(Turret turret, double speed) {
    double output = -Math.abs(speed);
    return Commands.run(() -> turret.setShootOpenLoop(output), turret)
        .finallyDo(() -> turret.shootStop())
        .withName("ShootNegative");
  }

  /** Manual control of shooter with joystick */
  public static Command shootManual(Turret turret, DoubleSupplier speed) {
    return Commands.run(() -> turret.setShootOpenLoop(speed.getAsDouble()), turret)
        .finallyDo(() -> turret.shootStop())
        .withName("ShootManual");
  }

  /** Stop shooter */
  public static Command stopShooter(Turret turret) {
    return Commands.runOnce(turret::shootStop, turret).withName("StopShooter");
  }

  // ==================== COMBINED COMMANDS ====================

  /** Spin up shooter and hold at velocity */
  public static Command spinUpAndHold(Turret turret, double velocityRPS) {
    return Commands.run(() -> turret.setShootVelocity(velocityRPS), turret)
        .withName("SpinUpAndHold");
  }

  /** Turn to position while spinning up shooter */
  public static Command aimAndSpinUp(Turret turret, double targetRotations, double velocityRPS) {
    return Commands.parallel(
            turnToPosition(turret, targetRotations), spinUpAndHold(turret, velocityRPS))
        .withName("AimAndSpinUp");
  }

  /** Stop all turret motors */
  public static Command stopAll(Turret turret) {
    return Commands.runOnce(
            () -> {
              turret.turnStop();
              turret.shootStop();
            },
            turret)
        .withName("StopAll");
  }
}
