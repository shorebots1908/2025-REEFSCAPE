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
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
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
  private final CommandXboxController player1 = new CommandXboxController(0);
  private final CommandXboxController player2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = initDrive();
    vision = initVision();
    elevator = initElevator(new ElevatorConfig(9, 10, 1.0, 0.0, 0.0, 0.0, 64.5));
    coralIntake =
        initIntake(
            new IntakeConfig(
                "Coral",
                11,
                12,
                19,
                0,
                3335,
                6.0,
                0.001,
                0.0,
                0.5,
                0.313,
                5.125,
                false,
                true,
                IntakeCommands.CORAL_WRIST_STOW));
    algaeIntake =
        initIntake(
            new IntakeConfig(
                "Algae",
                13,
                14,
                17,
                1,
                3335,
                7.0,
                0.001,
                0.0,
                1.0,
                0.371,
                0.681,
                true,
                false,
                IntakeCommands.ALGAE_WRIST_STOW));
    climber = initClimber(new ClimberConfig(15, 16, 5.0, 0.0, 0.0, 0.0, 67.95));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
    configureAutoCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configurePlayer1();
    configurePlayer2();
  }

  private void configurePlayer1() {
    // Drive commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -player1.getLeftY(),
            () -> -player1.getLeftX(),
            () -> -player1.getRightX()));

    player1.start().onTrue(Commands.runOnce(drive::gyroReset, drive));

  

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Elevator commands
    // controller
    //     .rightBumper()
    //     .whileTrue(ElevatorCommands.moveByJoystick(elevator, () -> controller.getRightY()));
    // controller.povDown().onTrue(ElevatorCommands.goToPosition(elevator,
    // ElevatorCommands.BOTTOM));
    player1
        .povLeft()
        .and(player1.leftBumper().negate())
        .and(player1.rightBumper().negate())
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L2));
    player1
        .povRight()
        .and(player1.leftBumper().negate())
        .and(player1.rightBumper().negate())
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L3));
    player1
        .povUp()
        .and(player1.leftBumper().negate())
        .and(player1.rightBumper().negate())
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L4));
    // Elevator down with stow
    // player1
    //     .povDown()
    //     .and(player1.leftBumper().negate())
    //     .and(player1.rightBumper().negate())
    //     .onTrue(
    //         IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW)
    //             .alongWith(
    //                 IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_STOW),
    //                 ElevatorCommands.goToPosition(elevator, ElevatorCommands.BOTTOM)));

    // Elevator down, no stow
    player1
        .povDown()
        .and(player1.leftBumper().negate())
        .and(player1.rightBumper().negate())
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.BOTTOM));

    // Algae set positions
    // player1
    //     .leftBumper()
    //     .and(player1.povUp())
    //     .onTrue(
    //         ElevatorCommands.goToPosition(elevator, ElevatorCommands.ALGAE_PROC)
    //             .alongWith(
    //                 IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_STOW),
    //                 IntakeCommands.goToPosition(algaeIntake,
    // IntakeCommands.ALGAE_WRIST_DEPLOY)));
    // player1
    //     .leftBumper()
    //     .and(player1.povDown())
    //     .onTrue(IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW));
    // player1
    //     .leftBumper()
    //     .and(player1.povRight())
    //     .onTrue(
    //         ElevatorCommands.goToPosition(elevator, ElevatorCommands.ALGAE_L3)
    //             .alongWith(
    //                 IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_STOW),
    //                 IntakeCommands.goToPosition(algaeIntake,
    // IntakeCommands.ALGAE_WRIST_DEPLOY)));
    // player1
    //     .leftBumper()
    //     .and(player1.povLeft())
    //     .onTrue(
    //         ElevatorCommands.goToPosition(elevator, ElevatorCommands.ALGAE_L2)
    //             .alongWith(
    //                 IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_STOW),
    //                 IntakeCommands.goToPosition(algaeIntake,
    // IntakeCommands.ALGAE_WRIST_DEPLOY)));

    // player1
    //     .rightBumper()
    //     .and(player1.povUp())
    //     .onTrue(
    //         ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L4)
    //             .alongWith(
    //                 IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW),
    //                 IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE)));
    // player1
    //     .rightBumper()
    //     .and(player1.povDown())
    //     .onTrue(
    //         ElevatorCommands.goToPosition(elevator, ElevatorCommands.BOTTOM)
    //             .alongWith(
    //                 IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_INTAKE),
    //                 IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW)));
    // player1
    //     .rightBumper()
    //     .and(player1.povRight())
    //     .onTrue(
    //         ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L3)
    //             .alongWith(
    //                 IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW),
    //                 IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE)));
    // player1
    //     .rightBumper()
    //     .and(player1.povLeft())
    //     .onTrue(
    //         ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L2)
    //             .alongWith(
    //                 IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW),
    //                 IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE)));

    // Coral commands
    // controller.a().onTrue(IntakeCommands.goToPosition(coralIntake, new BasePosition(0.5)));
    // controller.y().onTrue(IntakeCommands.goToPosition(coralIntake, new BasePosition(1.0)));
    // controller.x().onTrue(IntakeCommands.goToPosition(algaeIntake, new BasePosition(0.5)));
    // controller.b().onTrue(IntakeCommands.goToPosition(algaeIntake, new BasePosition(1.0)));
    // player1
    //    .a()
    //    .whileTrue(
    // player1
    //    .b()
    //    .whileTrue(IntakeCommands.setTargetPosition(algaeIntake,
    // IntakeCommands.ALGAE_WRIST_STOW));

    player1.b().whileTrue(IntakeCommands.feedIn(coralIntake));
    player1.a().whileTrue(IntakeCommands.feedOut(coralIntake));

    player1
        .x()
        .onTrue(IntakeCommands.setTargetPosition(coralIntake, IntakeCommands.CORAL_WRIST_INTAKE));
    player1
        .y()
        .onTrue(IntakeCommands.setTargetPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE));
  }

  // Manual climber commands
  //   player1
  //       .rightTrigger(0.5)
  //       .whileTrue(ClimberCommands.joystick(climber, () -> player1.getRightY()))
  //       .onFalse(ClimberCommands.joystick(climber, () -> 0.0));
  //

  private void configurePlayer2() {
    // // Manual coral commands
    // copied commands from player1 -MZ
    player2
        .povLeft()
        .and(player2.leftBumper().negate())
        .and(player2.rightBumper().negate())
        // .and(player2.rightTrigger(0.5).negate())
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L2));
    player2
        .povRight()
        .and(player2.leftBumper().negate())
        .and(player2.rightBumper().negate())
        // .and(player2.rightTrigger(0.5).negate())
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L3));
    player2
        .povUp()
        .and(player2.leftBumper().negate())
        .and(player2.rightBumper().negate())
        // .and(player2.rightTrigger(0.5).negate())
        .onTrue(
            ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L4)
                .andThen(
                    IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE)));
    player2
        .povDown()
        .and(player2.leftBumper().negate())
        .and(player2.rightBumper().negate())
        // .and(player2.rightTrigger(0.5).negate())
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.BOTTOM));

    // player2
    //     .back()
    //     .onTrue(IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_DEPLOY));

    // coralIntake.setDefaultCommand(
    //     IntakeCommands.moveByJoystick(coralIntake, () -> -player2.getLeftY() * 0.5, () -> 0.0));
    player2
        .leftTrigger(0.5)
        .whileTrue(
            IntakeCommands.moveByJoystick(
                coralIntake,
                () -> -MathUtil.applyDeadband(player2.getLeftY(), 0.07) * 0.5,
                () -> 0.0))
        .onFalse(IntakeCommands.moveByJoystick(coralIntake, () -> 0.0, () -> 0.0));
    // Without right trigger, X and A feed coral
    player2
        .x()
        .and(player2.rightTrigger(0.5).negate())
        .whileTrue(IntakeCommands.feedIn(coralIntake));
    player2
        .a()
        .and(player2.rightTrigger(0.5).negate())
        .whileTrue(IntakeCommands.feedOut(coralIntake));
    player2.a().and(player2.rightTrigger(0.5)).whileTrue(IntakeCommands.feedOut(coralIntake, 0.15));

    // With right trigger, X and A set algae wrist
    // Algae wrist disabled at 3/9/25 competition
    // player2
    //     .x()
    //     .and(player2.rightTrigger(0.5))
    //     .onTrue(IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW));
    // player2
    //     .a()
    //     .and(player2.rightTrigger(0.5))
    //     .onTrue(IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_DEPLOY));

    // algaeIntake.setDefaultCommand(
    //     IntakeCommands.moveByJoystick(algaeIntake, () -> -player2.getRightY() * 0.5, () -> 0.0));
    // player2
    //     .rightTrigger(0.5)
    //     .and(player2.back().negate())
    //     .whileTrue(
    //         // removed deadbnd code, didn't seem to work -MZ
    //         IntakeCommands.moveByJoystick(
    //             algaeIntake, () -> -MathUtil.applyDeadband(player2.getRightY(), 0.07), () ->
    // 0.0))
    //     .onFalse(IntakeCommands.moveByJoystick(algaeIntake, () -> 0.0, () -> 0.0));
    //    IntakeCommands.moveByJoystick(algaeIntake, () -> -player2.getRightY(), () -> 0.0))
    // .onFalse(IntakeCommands.moveByJoystick(algaeIntake, () -> 0.0, () -> 0.0));

    // player2
    //     .rightTrigger(0.5)
    //     .and(player2.povUp())
    //     .onTrue(
    //         new RepeatCommand(ElevatorCommands.goToPosition(elevator,
    // ElevatorCommands.ALGAE_L3)));
    // player2
    //     .rightTrigger(0.5)
    //     .and(player2.povRight())
    //     .onTrue(
    //         new RepeatCommand(ElevatorCommands.goToPosition(elevator,
    // ElevatorCommands.ALGAE_L2)));
    // player2
    //     .rightTrigger(0.5)
    //     .and(player2.povDown())
    //     .onTrue(
    //         new RepeatCommand(
    //             ElevatorCommands.goToPosition(elevator, ElevatorCommands.ALGAE_PROCESSOR)));
    // player2
    //     .rightTrigger(0.5)
    //     .and(player2.povLeft())
    //     .onTrue(
    //         new RepeatCommand(
    //             IntakeCommands.goToPosition(algaeIntake, IntakeCommands.ALGAE_WRIST_STOW)));

    // player2.y().whileTrue(IntakeCommands.feedIn(algaeIntake, 0.7));
    // player2.y().whileTrue(IntakeCommands.feedIn(algaeIntake, 0.6));
    // player2.b().whileTrue(IntakeCommands.feedOut(algaeIntake));

    player2
        .start()
        .onTrue(IntakeCommands.setTargetPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE));

    player2
        .leftBumper()
        .whileTrue(ElevatorCommands.moveByJoystick(elevator, () -> 0.3))
        .onFalse(ElevatorCommands.moveByJoystick(elevator, () -> 0.0));
    player2
        .rightBumper()
        .whileTrue(ElevatorCommands.moveByJoystick(elevator, () -> -0.3))
        .onFalse(ElevatorCommands.moveByJoystick(elevator, () -> 0.0));

    // player2
    //     .leftTrigger(0.5)
    //     .whileTrue(
    //         IntakeCommands.moveByJoystick(
    //             coralIntake,
    //             () -> -player2.getRightY(),
    //             () -> {
    //               if (player2.y().getAsBoolean()) {
    //                 return 5.0;
    //               }
    //               if (player2.a().getAsBoolean()) {
    //                 return -5.0;
    //               }
    //               return 0.0;
    //             }));

    // Manual climber commands
    // player2
    //     .back()
    //     .whileTrue(ClimberCommands.joystick(climber, () -> player2.getRightY()))
    //     .onFalse(ClimberCommands.joystick(climber, () -> 0.0));
  }

  private void configureAutoCommand(String name, Command command) {
    autoChooser.addOption(name, command);
    NamedCommands.registerCommand(name, command);
  }

  private void configureAutoCommands() {
    // // Set up SysId routines
    // configureAutoCommand(
    //     autoChooser,
    //     "drive-wheel-radius-characterization",
    //     DriveCommands.wheelRadiusCharacterization(drive));
    // configureAutoCommand(
    //     autoChooser,
    //     "drive-simple-ff-characterization",
    //     DriveCommands.feedforwardCharacterization(drive));
    // configureAutoCommand(
    //     autoChooser,
    //     "drive-sysid-quasistatic-forward",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // configureAutoCommand(
    //     autoChooser,
    //     "drive-sysid-quasistatic-reverse",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // configureAutoCommand(
    //     autoChooser,
    //     "drive-sysid-dynamic-forward",
    //     drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // configureAutoCommand(
    //     autoChooser,
    //     "drive-sysid-dynamic-reverse",
    //     drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // configureAutoCommand(
    //     "elevator-top", ElevatorCommands.setTargetPosition(elevator, new BasePosition(1.0)));
    // configureAutoCommand(
    //     "elevator-bottom", ElevatorCommands.setTargetPosition(elevator, new BasePosition(0.0)));
    // configureAutoCommand(
    //     "elevator-l4",
    //     Commands.sequence(
    //         IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE),
    //         ElevatorCommands.goToPosition(elevator, ElevatorCommands.CORAL_L4)));

    // configureAutoCommand("pickup", AutoCommands.pickup(coralIntake, elevator));
    // configureAutoCommand(
    //     "score-l1", AutoCommands.score(coralIntake, elevator, ElevatorCommands.BOTTOM));
    // configureAutoCommand(
    //     "score-l2", AutoCommands.score(coralIntake, elevator, ElevatorCommands.CORAL_L2));
    // configureAutoCommand(
    //     "score-l3", AutoCommands.score(coralIntake, elevator, ElevatorCommands.CORAL_L3));
    // configureAutoCommand(
    //     "score-l4", AutoCommands.score(coralIntake, elevator, ElevatorCommands.CORAL_L4));

    configureAutoCommand(
        "elevator-l1", ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.BOTTOM));
    configureAutoCommand(
        "elevator-l2", ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.CORAL_L2));
    configureAutoCommand(
        "elevator-l3", ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.CORAL_L3));
    configureAutoCommand(
        "elevator-l4", ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.CORAL_L4));

    configureAutoCommand("feed-in", IntakeCommands.feedIn(coralIntake));
    configureAutoCommand("feed-out", IntakeCommands.feedOut(coralIntake));
    configureAutoCommand("feed-spit", IntakeCommands.feedOut(coralIntake, 0.2));
    configureAutoCommand(
        "coral-accept",
        IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_INTAKE));
    configureAutoCommand(
        "coral-dispense",
        IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE));

    configureAutoCommand("auto1", new PathPlannerAuto("auto1"));
    configureAutoCommand("auto2", new PathPlannerAuto("auto2"));
    configureAutoCommand("auto3", new PathPlannerAuto("auto3"));
    configureAutoCommand("auto4", new PathPlannerAuto("auto4"));
    configureAutoCommand("midcompauto", new PathPlannerAuto("midcompauto"));
    configureAutoCommand("auto1path1", DriveCommands.followPath(drive, "auto1path1"));
    configureAutoCommand("auto1path2", DriveCommands.followPath(drive, "auto1path2"));
    configureAutoCommand("auto1path3", DriveCommands.followPath(drive, "auto1path3"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return DriveCommands.autoPath(drive);
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
            new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
            new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1),
            new VisionIOPhotonVision(VisionConstants.camera2Name, VisionConstants.robotToCamera2));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose));

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
