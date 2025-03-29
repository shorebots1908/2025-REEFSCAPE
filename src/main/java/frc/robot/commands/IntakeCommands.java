package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.*;

public class IntakeCommands {
  public static final double FEED_SPEED = 0.3;
  public static final double FEED_STICK = 0.1;
  public static final BasePosition CORAL_WRIST_DOWN = new BasePosition(0.0);
  public static final BasePosition CORAL_WRIST_STOW = new BasePosition(1.0);
  public static final BasePosition CORAL_WRIST_INTAKE = new BasePosition(0.75); // was 0.74
  public static final BasePosition CORAL_WRIST_SCORE = new BasePosition(0.21);
  public static final BasePosition CORAL_WRIST_L3 = new BasePosition(0.40);
  public static final BasePosition CORAL_WRIST_L4 = new BasePosition(0.3);

  // public static final BasePosition ALGAE_WRIST_STOW = new BasePosition(0.8);
  // public static final BasePosition ALGAE_WRIST_DEPLOY = new BasePosition(0.2);
  public static final BasePosition ALGAE_WRIST_STOW = new BasePosition(0.9);
  public static final BasePosition ALGAE_WRIST_DEPLOY = new BasePosition(0.128);

  public static Command feedHoldSticky(Intake intake) {
    return Commands.run(
        () -> {
          if (intake.isHolding()) {
            // If the sensor has a coral, keep feeding in to hold it
            intake.setFeedOpenLoop(FEED_STICK);
          } else {
            intake.feedStop();
          }
        },
        intake);
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
}
