package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;

public class VisionCommands {
    public static Command periodicVisionUpdate(Vision vision) {
        return Commands.run(() -> {
            vision.processCameraObservations();
        }, vision);
    }

    public static Command snapshotVisionUpdate(Vision vision) {
        return Commands.runOnce(() -> {
            vision.processCameraObservations();
        }, vision).andThen(Commands.idle(vision));
    }
}
