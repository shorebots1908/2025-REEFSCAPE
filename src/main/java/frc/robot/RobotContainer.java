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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.BasePosition;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConfig;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConfig;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
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
  private final Intake coralIntake;
  private final Intake algaeIntake;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = initDrive();
    vision = initVision();
    elevator = initElevator(new ElevatorConfig(9, 10, 1.0, 0.0, 0.0, 0.0, 63.7));
    coralIntake =
        initIntake(new IntakeConfig("Coral", 11, 12, 19, 10.0, 0.001, 0.0, 0.252, 0.646, true));
    algaeIntake =
        initIntake(new IntakeConfig("Algae", 13, 14, 17, 10.0, 0.001, 0.0, 0.378, 0.695, false));
    climber = initClimber(new ClimberConfig(15, 16, 5.0, 0.0, 0.0, 0.0, 45.0));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    configureAutoCommand(
        autoChooser,
        "drive-wheel-radius-characterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    configureAutoCommand(
        autoChooser,
        "drive-simple-ff-characterization",
        DriveCommands.feedforwardCharacterization(drive));
    configureAutoCommand(
        autoChooser,
        "drive-sysid-quasistatic-forward",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    configureAutoCommand(
        autoChooser,
        "drive-sysid-quasistatic-reverse",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    configureAutoCommand(
        autoChooser,
        "drive-sysid-dynamic-forward",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    configureAutoCommand(
        autoChooser,
        "drive-sysid-dynamic-reverse",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureAutoCommand(
        autoChooser,
        "Elevator Top",
        ElevatorCommands.setTargetPosition(elevator, new BasePosition(1.0)));
    configureAutoCommand(
        autoChooser,
        "Elevator Bottom",
        ElevatorCommands.setTargetPosition(elevator, new BasePosition(0.0)));

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
    // Drive commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.start().onTrue(Commands.runOnce(drive::gyroReset, drive));
    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .leftTrigger(0.5)
        .whileTrue(
            IntakeCommands.moveByJoystick(
                coralIntake,
                () -> -controller.getRightY(),
                () -> {
                  if (controller.y().getAsBoolean()) {
                    return 5.0;
                  }
                  if (controller.a().getAsBoolean()) {
                    return -5.0;
                  }
                  return 0.0;
                }));

    // Elevator commands
    // controller
    //     .rightBumper()
    //     .whileTrue(ElevatorCommands.moveByJoystick(elevator, () -> controller.getRightY()));
    controller.povDown().onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.BOTTOM));
    controller.povLeft().onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L2));
    controller
        .povRight()
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L3));
    controller.povUp().onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L4));
    controller
        .povDown()
        .and(controller.leftBumper().negate())
        .and(controller.rightBumper().negate())
        .onTrue(
            IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW)
                .alongWith(
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_STOW),
                    ElevatorCommands.goToPosition(elevator, ElevatorCommands.BOTTOM)));
    // algae set positions
    controller
        .leftBumper()
        .and(controller.povUp())
        .onTrue(
            ElevatorCommands.goToPosition(elevator, ElevatorCommands.ALGAE_PROC)
                .alongWith(
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_STOW),
                    IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_DEPLOY)));
    controller
        .leftBumper()
        .and(controller.povDown())
        .onTrue(IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW));
    controller
        .leftBumper()
        .and(controller.povRight())
        .onTrue(
            ElevatorCommands.goToPosition(elevator, ElevatorCommands.ALGAE_L3)
                .alongWith(
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_STOW),
                    IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_DEPLOY)));
    controller
        .leftBumper()
        .and(controller.povLeft())
        .onTrue(
            ElevatorCommands.goToPosition(elevator, ElevatorCommands.ALGAE_L2)
                .alongWith(
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_STOW),
                    IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_DEPLOY)));

    controller
        .rightBumper()
        .and(controller.povUp())
        .onTrue(
            ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L4)
                .alongWith(
                    IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW),
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE)));
    controller
        .rightBumper()
        .and(controller.povDown())
        .onTrue(
            ElevatorCommands.goToPosition(elevator, ElevatorCommands.BOTTOM)
                .alongWith(
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_INTAKE),
                    IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW)));
    controller
        .rightBumper()
        .and(controller.povRight())
        .onTrue(
            ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L3)
                .alongWith(
                    IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW),
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE)));
    controller
        .rightBumper()
        .and(controller.povLeft())
        .onTrue(
            ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L2)
                .alongWith(
                    IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW),
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE)));

    // Coral commands
    // controller.a().onTrue(IntakeCommands.goToPosition(coralIntake, new BasePosition(0.5)));
    // controller.y().onTrue(IntakeCommands.goToPosition(coralIntake, new BasePosition(1.0)));
    // controller.x().onTrue(IntakeCommands.goToPosition(algaeIntake, new BasePosition(0.5)));
    // controller.b().onTrue(IntakeCommands.goToPosition(algaeIntake, new BasePosition(1.0)));
    controller
        .a()
        .whileTrue(
            IntakeCommands.setTargetPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_DEPLOY));
    controller
        .b()
        .whileTrue(IntakeCommands.setTargetPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW));

    controller
        .x()
        .onTrue(IntakeCommands.setTargetPosition(coralIntake, IntakeCommands.CORAL_WRIST_INTAKE));
    controller
        .y()
        .onTrue(IntakeCommands.setTargetPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE));

    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         IntakeCommands.moveByJoystick(
    //             coralIntake,
    //             () -> -controller.getRightY(),
    //             () -> {
    //               if (controller.y().getAsBoolean()) {
    //                 return 5.0;
    //               }
    //               if (controller.a().getAsBoolean()) {
    //                 return -5.0;
    //               }
    //               return 0.0;
    //             }));

    controller
        .leftBumper()
        .whileTrue(
            IntakeCommands.moveByJoystick(
                algaeIntake,
                () -> -controller.getRightY(),
                () -> {
                  if (controller.y().getAsBoolean()) {
                    return 5.0;
                  }
                  if (controller.a().getAsBoolean()) {
                    return -5.0;
                  }
                  return 0.0;
                }));
  }

  private void configureAutoCommand(
      LoggedDashboardChooser<Command> chooser, String name, Command command) {
    chooser.addOption(name, command);
    NamedCommands.registerCommand(name, command);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return DriveCommands.autoPath(drive);
  }

  private Drive initDrive() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new Drive(
            new GyroIOPigeon2(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
      default:
        // Replayed robot, disable IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }
  }

  public Vision initVision() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));

      default:
        // Replayed robot, disable IO implementations
        return new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }
  }

  private Elevator initElevator(ElevatorConfig config) {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new Elevator(new ElevatorIOSparkMax(config));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Elevator(new ElevatorIOSim());

      default:
        // Replayed robot, disable IO implementations
        return new Elevator(new ElevatorIO() {});
    }
  }

  private Climber initClimber(ClimberConfig config) {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new Climber(new ClimberIOSparkMax(config));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Climber(new ClimberIOSim());

      default:
        // Replayed robot, disable IO implementations
        return new Climber(new ClimberIO() {});
    }
  }

  public Intake initIntake(IntakeConfig config) {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new Intake(new IntakeIOSparkMax(config));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Intake(new IntakeIOSim());

      default:
        // Replayed robot, disable IO implementations
        return new Intake(new IntakeIO() {});
    }
  }
}
