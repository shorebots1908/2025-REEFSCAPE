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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ClimberCommands;
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
import java.util.List;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
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
  private final List<Pose2d> faces;
  private final List<Pose2d> reefPoses;
  private final List<Pose2d> reefLeftPoses;
  private final List<Pose2d> reefRightPoses;
  private final List<Pose2d> intakePoses;

  // Controller
  private final CommandXboxController player1 = new CommandXboxController(0);
  private final CommandXboxController player2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = initDrive();
    vision = initVision();
    elevator = initElevator(new ElevatorConfig(9, 10, 1.0, 0.0, 0.0, 0.0, 68.0));
    coralIntake =
        initIntake(
            new IntakeConfig(
                "Coral",
                11,
                12,
                19,
                0,
                2000,
                0.3, // was 6.0
                0.0001, // was 0.001
                3, // was 0.0
                0.5,
                0.58,
                3.1,
                true,
                false,
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

    faces = AlignCommands.reefFaces();
    reefPoses =
        faces.stream()
            .flatMap((face) -> AlignCommands.faceToReefPair(face).stream())
            .collect(Collectors.toList());
    var reefPosesArray = reefPoses.toArray(new Pose2d[reefPoses.size()]);
    Logger.recordOutput("RobotContainer/reefPoses", reefPosesArray);

    reefLeftPoses =
        List.of(
            // To tweak individual poses, add offsets with offsetPose
            AlignCommands.offsetPose(reefPoses.get(0), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(2), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(4), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(6), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(8), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(10), 0.0, 0.0));
    var reefLeftPosesArray = reefLeftPoses.toArray(new Pose2d[reefLeftPoses.size()]);
    Logger.recordOutput("RobotContainer/reefLeftPoses", reefLeftPosesArray);

    reefRightPoses =
        List.of(
            // To tweak individual poses, add offsets with offsetPose
            AlignCommands.offsetPose(reefPoses.get(1), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(3), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(5), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(7), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(9), 0.0, 0.0),
            AlignCommands.offsetPose(reefPoses.get(11), 0.0, 0.0));
    var reefRightPosesArray = reefRightPoses.toArray(new Pose2d[reefRightPoses.size()]);
    Logger.recordOutput("RobotContainer/reefRightPoses", reefRightPosesArray);

    intakePoses =
        AlignCommands.intakeStationPoses().stream()
            .map((station) -> AlignCommands.offsetIntakePose(station))
            .collect(Collectors.toList());
    var intakePoseArray = intakePoses.toArray(new Pose2d[intakePoses.size()]);
    Logger.recordOutput("RobotContainer/intakePoses", intakePoseArray);

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

    // Align commands
    player1.leftTrigger().whileTrue(new AlignCommands.ToClosestPose(drive, reefPoses));
    player1.leftBumper().whileTrue(new AlignCommands.ToClosestPose(drive, intakePoses));

    // Elevator auto positions on the D-pad
    player1
        .povDown()
        .and(player1.leftBumper().negate())
        .and(player1.rightBumper().negate())
        .onTrue(ElevatorCommands.goToPosition(elevator, ElevatorCommands.BOTTOM));
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

    // Coral feeding
    player1.b().whileTrue(IntakeCommands.feedIn(coralIntake));
    player1.a().whileTrue(IntakeCommands.feedOut(coralIntake));

    // Coral wrist to Intake or Score positions
    player1.leftTrigger().whileTrue(new AlignCommands.ToClosestPose(drive, reefLeftPoses));
    player1.rightTrigger().whileTrue(new AlignCommands.ToClosestPose(drive, reefRightPoses));
    player1.leftBumper().whileTrue(new AlignCommands.ToClosestPose(drive, intakePoses));

    player1
        .x()
        .onTrue(IntakeCommands.setTargetPosition(coralIntake, IntakeCommands.CORAL_WRIST_INTAKE));
    player1
        .y()
        .onTrue(IntakeCommands.setTargetPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE));

    // Manual climber commands
    player2
        .rightTrigger(0.5)
        .whileTrue(ClimberCommands.joystick(climber, () -> -player2.getRightY()))
        .onFalse(ClimberCommands.joystick(climber, () -> 0.0));
  }

  private void configurePlayer2() {
    coralIntake.setDefaultCommand(IntakeCommands.feedHoldSticky(coralIntake));

    // Elevator auto positions on the D-pad
    player2
        .povDown()
        .and(player2.leftBumper().negate())
        .and(player2.rightBumper().negate())
        .onTrue(AutoCommands.smartElevatordown(elevator, coralIntake, ElevatorCommands.BOTTOM));
    player2
        .povLeft()
        .and(player2.leftBumper().negate())
        .and(player2.rightBumper().negate())
        .onTrue(AutoCommands.smartElevatorl3(elevator, coralIntake, ElevatorCommands.CORAL_L3));
    player2
        .povRight()
        .and(player2.leftBumper().negate())
        .and(player2.rightBumper().negate())
        .onTrue(AutoCommands.smartElevatorl3(elevator, coralIntake, ElevatorCommands.CORAL_L3));
    player2
        .povUp()
        .and(player2.leftBumper().negate())
        .and(player2.rightBumper().negate())
        .onTrue(AutoCommands.smartElevator(elevator, coralIntake, ElevatorCommands.CORAL_L4));

    // Hold left trigger to manually control coral wrist with LeftY joystick
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

    player2.b().whileTrue(IntakeCommands.feedOut(coralIntake, 0.2));

    player2.a().and(player2.rightTrigger(0.5)).whileTrue(IntakeCommands.feedOut(coralIntake, 0.15));

    // Start button moves coral wrist to Score
    player2
        .start()
        .onTrue(IntakeCommands.setTargetPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE));

    // Manual elevator up and down on bumpers
    player2
        .leftBumper()
        .whileTrue(ElevatorCommands.moveByJoystick(elevator, () -> 0.5))
        .onFalse(ElevatorCommands.moveByJoystick(elevator, () -> 0.0));
    player2
        .rightBumper()
        .whileTrue(ElevatorCommands.moveByJoystick(elevator, () -> -0.5))
        .onFalse(ElevatorCommands.moveByJoystick(elevator, () -> 0.0));
  }

  private void configureAutoCommand(String name, Command command) {
    autoChooser.addOption(name, command);
    NamedCommands.registerCommand(name, command);
  }

  private void configureAutoCommands() {
    configureAutoCommand(
        "elevator-l1", ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.BOTTOM));
    configureAutoCommand(
        "elevator-l2", ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.CORAL_L2));
    configureAutoCommand(
        "elevator-l3", ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.CORAL_L3));
    configureAutoCommand(
        "elevator-l4", ElevatorCommands.setTargetPosition(elevator, ElevatorCommands.CORAL_L4));

    configureAutoCommand("feed-in", IntakeCommands.feedIn(coralIntake).withTimeout(3.0));
    configureAutoCommand(
        "feed-in-until-holding", IntakeCommands.feedInUntilHolding(coralIntake).withTimeout(4));
    configureAutoCommand("feed-out", IntakeCommands.feedOut(coralIntake, 0.5).withTimeout(0.3));
    configureAutoCommand("feed-spit", IntakeCommands.feedOut(coralIntake, 0.2).withTimeout(0.3));
    configureAutoCommand(
        "coral-accept",
        IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_INTAKE));
    configureAutoCommand(
        "coral-dispense",
        IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_SCORE));
    configureAutoCommand(
        "coral-down", IntakeCommands.goToPosition(coralIntake, IntakeCommands.CORAL_WRIST_DOWN));
    configureAutoCommand(
        "align-to-reef", new AlignCommands.ToClosestPose(drive, reefPoses).withTimeout(1));
    configureAutoCommand(
        "align-to-intake", new AlignCommands.ToClosestPose(drive, intakePoses).withTimeout(1));

    configureAutoCommand("auto1", new PathPlannerAuto("auto1"));
    configureAutoCommand("auto2", new PathPlannerAuto("auto2"));
    configureAutoCommand("auto3", new PathPlannerAuto("auto3"));
    configureAutoCommand("auto4", new PathPlannerAuto("auto4"));
    configureAutoCommand("midcompauto", new PathPlannerAuto("midcompauto"));
    configureAutoCommand("2pieceauto", new PathPlannerAuto("2pieceauto"));
    configureAutoCommand("2pieceauto2", new PathPlannerAuto("2pieceauto2"));
    configureAutoCommand("1piecemiddleauto", new PathPlannerAuto("1piecemiddleauto"));
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
