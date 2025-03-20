package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.DoubleSupplier;

public class WristCommands {
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

  public static Command moveByJoystick(Wrist wrist, DoubleSupplier wristPosition) {
    return Commands.run(
        () -> {
          wrist.setWristOpenLoop(wristPosition.getAsDouble());
        },
        wrist);
  }

  public static Command pickup(Intake intake, Wrist wrist) {
    return Commands.sequence(
        goToPosition(wrist, CORAL_WRIST_INTAKE), IntakeCommands.feedIn(intake));
  }

  public static Command place(Intake intake, Wrist wrist) {
    return Commands.sequence(
        goToPosition(wrist, CORAL_WRIST_SCORE), IntakeCommands.feedOut(intake));
  }

  public static Command setTargetPosition(Wrist wrist, BasePosition position) {
    return Commands.runOnce(
        () -> {
          wrist.setTargetPosition(position);
        },
        wrist);
  }

  public static Command waitUntilCoralAtTargetPosition(Wrist wrist) {
    return Commands.idle(wrist)
        .until(wrist::atTargetPosition)
        .withName("WaitUntilCoralAtTargetPosition");
  }

  public static Command goToPosition(Wrist wrist, BasePosition position) {
    return Commands.sequence(
        setTargetPosition(wrist, position), waitUntilCoralAtTargetPosition(wrist));
  }
}
