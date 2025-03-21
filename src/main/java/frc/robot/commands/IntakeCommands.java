package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  public static final double FEED_SPEED = 0.3;
  public static final BasePosition CORAL_WRIST_DOWN = new BasePosition(0.0);
  public static final BasePosition CORAL_WRIST_STOW = new BasePosition(1.0);
  public static final BasePosition CORAL_WRIST_INTAKE = new BasePosition(0.78); // was 0.74
  public static final BasePosition CORAL_WRIST_SCORE = new BasePosition(0.21);
  public static final BasePosition CORAL_WRIST_L3 = new BasePosition(0.40);
  public static final BasePosition CORAL_WRIST_L4 = new BasePosition(0.3);

  // public static final BasePosition ALGAE_WRIST_STOW = new BasePosition(0.8);
  // public static final BasePosition ALGAE_WRIST_DEPLOY = new BasePosition(0.2);
  public static final BasePosition ALGAE_WRIST_STOW = new BasePosition(0.9);
  public static final BasePosition ALGAE_WRIST_DEPLOY = new BasePosition(0.128);

  public static Command moveByJoystick(Intake intake, DoubleSupplier wrist, DoubleSupplier wheels) {
    return Commands.run(
        () -> {
          intake.setWristOpenLoop(wrist.getAsDouble());
          intake.setFeedOpenLoop(wheels.getAsDouble());
        },
        intake);
  }

  public static Command pickup(Intake intake) {
    return Commands.sequence(goToPosition(intake, CORAL_WRIST_INTAKE), feedIn(intake));
  }

  public static Command place(Intake intake) {
    return Commands.sequence(goToPosition(intake, CORAL_WRIST_SCORE), feedOut(intake));
  }

  public static Command feedIn(Intake intake) {
    return feedIn(intake, FEED_SPEED);
  }

  public static Command feedInUntilHolding(Intake intake) {
    return feedIn(intake).until(intake::isHolding);
  }

  public static Command feedIn(Intake intake, double feedSpeed) {
    // Absolute value of feedSpeed
    double speed = feedSpeed > 0 ? feedSpeed : -feedSpeed;
    return Commands.run(
            () -> {
              intake.setFeedOpenLoop(speed);
            },
            intake)
        .finallyDo(
            () -> {
              intake.feedStop();
            });
  }

  public static Command feedOut(Intake intake) {
    // Negation happens inside this call
    return feedOut(intake, FEED_SPEED);
  }

  public static Command feedOut(Intake intake, double feedSpeed) {
    // Absolute value of feedSpeed
    double speed = feedSpeed > 0 ? feedSpeed : -feedSpeed;
    return Commands.run(
            () -> {
              intake.setFeedOpenLoop(-speed);
            },
            intake)
        .finallyDo(
            () -> {
              intake.feedStop();
            });
  }

  public static Command setTargetPosition(Intake intake, BasePosition position) {
    return Commands.runOnce(
        () -> {
          intake.setTargetPosition(position);
        },
        intake);
  }

  public static Command isHolding(Intake intake) {
    return Commands.run(
        () -> {
          intake.isHolding();
        },
        intake);
  }

  public static Command waitUntilCoralAtTargetPosition(Intake intake) {
    return Commands.idle(intake)
        .until(intake::atTargetPosition)
        .withName("WaitUntilCoralAtTargetPosition");
  }

  public static Command waitUntilCoralIsHolding(Intake intake, boolean holding) {
    return Commands.idle(intake)
        .until(
            () -> {
              if (holding) {
                return intake.isHolding();
              } else {
                return !intake.isHolding();
              }
            })
        .withName("WaitUntilCoralIsHolding");
  }

  public static Command goToPosition(Intake intake, BasePosition position) {
    return Commands.sequence(
        setTargetPosition(intake, position), waitUntilCoralAtTargetPosition(intake));
  }
}
