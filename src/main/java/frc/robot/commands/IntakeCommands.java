package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  public static final double FEED_SPEED = 0.3;
  public static final BasePosition CORAL_WRIST_STOW = new BasePosition(1.0);
  public static final BasePosition CORAL_WRIST_INTAKE = new BasePosition(0.9);
  public static final BasePosition CORAL_WRIST_SCORE = new BasePosition(0.35);

  public static final BasePosition ALGAE_WRIST_STOW = new BasePosition(1.0);
  public static final BasePosition ALGAE_WRIST_DEPLOY = new BasePosition(0.0);

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
    return Commands.run(
            () -> {
              intake.setFeedOpenLoop(-FEED_SPEED);
            },
            intake)
        .finallyDo(
            () -> {
              intake.feedStop();
            });
  }

  public static Command feedOut(Intake intake) {
    return Commands.run(
            () -> {
              intake.setFeedOpenLoop(FEED_SPEED);
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
