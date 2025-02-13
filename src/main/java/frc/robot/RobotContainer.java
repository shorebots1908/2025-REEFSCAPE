// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CoralToolCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.ballerIntake.BallerIntake;
import frc.robot.subsystems.ballerIntake.BallerIntakeConfig;
import frc.robot.subsystems.ballerIntake.BallerIntakeIO;
import frc.robot.subsystems.ballerIntake.BallerIntakeIOSim;
import frc.robot.subsystems.ballerIntake.BallerIntakeIOSparkMax;
import frc.robot.subsystems.coralTool.CoralTool;
import frc.robot.subsystems.coralTool.CoralToolConfig;
import frc.robot.subsystems.coralTool.CoralToolIO;
import frc.robot.subsystems.coralTool.CoralToolIOSim;
import frc.robot.subsystems.coralTool.CoralToolIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConfig;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final BallerIntake ballerIntake;
  private final CoralTool coralTool;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                // new GyroIOADIS16470(),
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        // new VisionIOPhotonVision(
        //     VisionConstants.camera1Name, VisionConstants.robotToCamera1));

        elevator =
            new Elevator(
                new ElevatorIOSparkMax(
                    new ElevatorConfig(
                        // Motor IDs: left, right
                        9, 10,

                        // TODO LIMIT SWITCHES DIGITAL IDs
                        //
                        // Limit switches: lower, upper
                        0, 0,

                        // TODO MEASURE SANE DEFAULTS
                        //
                        // Encoder range values: lower, upper
                        0, 100)));

        ballerIntake = new BallerIntake(new BallerIntakeIOSparkMax(new BallerIntakeConfig(13, 14)));

        coralTool = new CoralTool(new CoralToolIOSparkMax(new CoralToolConfig(11, 12, 19)));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        elevator = new Elevator(new ElevatorIOSim());

        ballerIntake = new BallerIntake(new BallerIntakeIOSim());

        coralTool = new CoralTool(new CoralToolIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        ballerIntake = new BallerIntake(new BallerIntakeIO() {});

        coralTool = new CoralTool(new CoralToolIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // controller.y().onTrue(ElevatorCommands.moveToPosition(null, null));
    // controller.povUp().onTrue(CoralToolCommands.coralPickup(coralIntake));
    // controller.povDown().onTrue(CoralToolCommands.coralPlace(coralIntake));

    // NOTE:
    // Value gets applied as motor.setVoltage();
    // Which means the range is maybe -12 - 12
    // The joystick gives us -1, 1
    controller
        .rightBumper()
        .whileTrue(ElevatorCommands.moveByJoystick(elevator, () -> controller.getRightY()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    controller
        .rightBumper()
        .onTrue(
            DriveCommands.joystickDriveRobotRelative(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller.y().onTrue(Commands.runOnce(drive::gyroResetY, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // While holding the left bumper, use right Y for coral wrist
    controller
        .leftBumper()
        .whileTrue(CoralToolCommands.moveByJoystick(coralTool, () -> -controller.getRightY(),() -> {
            if(controller.y().getAsBoolean()){
                return 5.0;
            }
            if(controller.a().getAsBoolean()){
                return -5.0;
            }
            return 0.0;
        }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return DriveCommands.autoPath(drive);
    // return DriveCommands.goToFieldPoint(drive, FieldPoint.REEF_AB);
  }
}
