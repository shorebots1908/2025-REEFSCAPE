package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.DoubleSupplier;

public class WristCommands {
  public static final double FEED_SPEED = 0.3;
  public static final BasePosition CORAL_WRIST_DOWN = new BasePosition(0.0);
  public static final BasePosition CORAL_WRIST_STOW = new BasePosition(1.0);
  // this is where the wrist angles are stored now
  public static final BasePosition CORAL_WRIST_INTAKE = new BasePosition(0.66); // was 0.74
  public static final BasePosition CORAL_WRIST_SCORE = new BasePosition(0.18);

  public static final BasePosition CORAL_WRIST_L3 = new BasePosition(0.34);
  public static final BasePosition CORAL_WRIST_L4 = new BasePosition(0.3);

  // public static final BasePosition ALGAE_WRIST_STOW = new BasePosition(0.8);
  // public static final BasePosition ALGAE_WRIST_DEPLOY = new BasePosition(0.2);
  public static final BasePosition ALGAE_WRIST_STOW = new BasePosition(0.0);
  public static final BasePosition ALGAE_WRIST_HALF = new BasePosition(0.5);
  public static final BasePosition ALGAE_WRIST_FULL = new BasePosition(1);

  public static Command moveByJoystick(
      Wrist wrist, DoubleSupplier wristPosition, DoubleSupplier wheels) {
    return Commands.run(
        () -> {
          wrist.setWristOpenLoop(wristPosition.getAsDouble());
        },
        wrist);
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

  public static Command waitUntilAlgaeatTargetPosition(Wrist wrist) {
    return Commands.idle(wrist)
        .until(wrist::atTargetPosition)
        .withName("WaitUntilAlgaeAtTargetPosition");
  }

  public static Command goToPosition(Wrist wrist, BasePosition position) {
    return Commands.sequence(
        setTargetPosition(wrist, position), waitUntilCoralAtTargetPosition(wrist));
  }
}
