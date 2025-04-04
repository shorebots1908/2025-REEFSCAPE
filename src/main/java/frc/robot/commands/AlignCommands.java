package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

/**
 * AlignCommands contains static functions for constructing a Command that will drive the Robot to a
 * Pose2d using three profiled PID controllers.
 *
 * <p>The overloaded functions allow us to set defaults for various inputs needed by the actual
 * command, which is constructed in the last function at the bottom. To override defaults, just call
 * the function that lets you pick all the inputs.
 */
public class AlignCommands {
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** The ProfiledPIDControllers we use need inputs for P, I, D, Velocity, and Acceleration */
  public static class AlignConfig {
    public double p;
    public double i;
    public double d;
    public double velocity;
    public double acceleration;

    public AlignConfig(double p, double i, double d, double velocity, double acceleration) {
      this.p = p;
      this.i = i;
      this.d = d;
      this.velocity = velocity;
      this.acceleration = acceleration;
    }
  }

  // allowable setpoint deviation
  public static final double ROTATION_TOLERANCE = 13; // degrees
  public static final double TRANSLATION_TOLERANCE = 0.08; // meters

  // ALIGN:

  public static final double ROTATION_P = 0.25; // was 0.25  then 0.9 w/ 10 degree tolerance
  public static final double ROTATION_I = 0.001; // was 0.0001
  public static final double ROTATION_D = 0.1;
  // try not to touch
  public static final double ROTATION_VELOCITY = 2.0;
  public static final double ROTATION_ACCEL = 50;

  // POSITION:

  public static final double TRANSLATION_P = 3.0;
  public static final double TRANSLATION_I = 0.0;
  public static final double TRANSLATION_D = 0.0;
  // try not to touch
  public static final double TRANSLATION_VELOCITY = 15;
  public static final double TRANSLATION_ACCEL = 50;

  /**
   * Given a Drive subsystem and a target Pose2d, this command will use three profiled PID
   * controllers to drive the robot to the pose.
   */
  public static Command alignToPose(Drive drive, Pose2d targetPose) {
    // AlignConfig is P, I, D, Velocity, Acceleration
    var translationConfig =
        new AlignConfig(
            TRANSLATION_P,
            TRANSLATION_I,
            TRANSLATION_D,
            TRANSLATION_VELOCITY,
            TRANSLATION_ACCEL); // p-1.6 i-0.0001 d-0.1
    var rotationConfig =
        new AlignConfig(ROTATION_P, ROTATION_I, ROTATION_D, ROTATION_VELOCITY, ROTATION_ACCEL);
    return alignToPose(drive, targetPose, translationConfig, rotationConfig);
  }

  /**
   * Provide an AlignConfig with custom gains for P, I, D, and contraints for Velocity and
   * Acceleration, and get a Command that drives with those characteristics.
   */
  public static Command alignToPose(
      Drive drive, Pose2d targetPose, AlignConfig translationConfig, AlignConfig rotationConfig) {
    var xPid =
        new ProfiledPIDController(
            translationConfig.p,
            translationConfig.i,
            translationConfig.d,
            new TrapezoidProfile.Constraints(
                translationConfig.velocity, translationConfig.acceleration));
    xPid.setTolerance(TRANSLATION_TOLERANCE);
    var yPid =
        new ProfiledPIDController(
            translationConfig.p,
            translationConfig.i,
            translationConfig.d,
            new TrapezoidProfile.Constraints(
                translationConfig.velocity, translationConfig.acceleration));
    yPid.setTolerance(TRANSLATION_TOLERANCE);
    var rPid =
        new ProfiledPIDController(
            rotationConfig.p,
            rotationConfig.i,
            rotationConfig.d,
            new TrapezoidProfile.Constraints(rotationConfig.velocity, rotationConfig.acceleration));
    rPid.enableContinuousInput(-Math.PI, Math.PI);
    rPid.setTolerance(Units.degreesToRadians(ROTATION_TOLERANCE));

    return alignToPose(drive, targetPose, xPid, yPid, rPid);
  }

  /**
   * Provide the actual ProfiledPIDControllers to use for alignment, and get a Command that drives
   * using those controllers.
   */
  public static Command alignToPose(
      Drive drive,
      Pose2d targetPose,
      ProfiledPIDController xPid,
      ProfiledPIDController yPid,
      ProfiledPIDController rPid) {

    return new FunctionalCommand(
        // initialize
        () -> {
          var initialPose = drive.getPose();
          xPid.reset(initialPose.getX());
          yPid.reset(initialPose.getY());
          rPid.reset(initialPose.getRotation().getRadians());
        },
        // execute
        () -> {
          var drivePose = drive.getPose();

          var xOut = xPid.calculate(drivePose.getX(), targetPose.getX());
          var yOut = yPid.calculate(drivePose.getY(), targetPose.getY());
          var rOut =
              -rPid.calculate(
                  drivePose.getRotation().getRadians(), targetPose.getRotation().getRadians());
          var fieldSpeeds = new ChassisSpeeds(xOut, yOut, rOut);
          var driveSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, drivePose.getRotation());
          drive.runVelocity(driveSpeeds);
        },
        // end(interrupted)
        (interrupted) -> {},
        // isFinished -> boolean
        () -> {
          return xPid.atGoal() && yPid.atGoal() && rPid.atGoal();
        },
        drive);
  }

  public static double TAU = Math.PI * 2;

  /** Generate the reef face poses using the default geometry. */
  public static List<Pose2d> reefFaces() {

    // Geometry setup: Need a center point and radius for the reef circle
    var reefCenterBlue = new Pose2d(4.4895, 4.026, Rotation2d.fromRadians(0.0));
    var reefCenterRed = new Pose2d(13.057, 4.026, Rotation2d.fromRadians(0.0));

    var alliance = DriverStation.getAlliance();
    var reefRadius = 1.2;

    if (alliance.isPresent()) {

      if (alliance.get() == Alliance.Blue) {
        Logger.recordOutput("ReefFaceIsBlue", true);
        return reefFaces(reefCenterBlue, reefRadius);

      } else if (alliance.get() == Alliance.Red) {
        Logger.recordOutput("ReefFaceIsBlue", false);
        return reefFaces(reefCenterRed, reefRadius);
      }
    }

    return reefFaces(reefCenterBlue, reefRadius);
  }

  /**
   * Reef faces are poses that sit in front of each of the hex faces of the reef. The rotation of
   * each pose aligns to a vector from the face point to the reef center.
   */
  public static List<Pose2d> reefFaces(Pose2d reefCenter, double reefRadius) {
    // Divide a circle around the reef into 6, one for each face
    var faces = 6;
    var faceArc = TAU / faces;

    List<Pose2d> reefFaces = new ArrayList<>();
    for (int i = 0; i < faces; i++) {
      var faceTheta = faceArc * i;
      var faceX = reefRadius * Math.cos(faceTheta) + reefCenter.getX();
      var faceY = reefRadius * Math.sin(faceTheta) + reefCenter.getY();

      // To get a vector pointing from face to center, flip theta by adding PI
      var faceToCenterTheta = Rotation2d.fromRadians(faceTheta + Math.PI);
      var facePose = new Pose2d(faceX, faceY, faceToCenterTheta);
      reefFaces.add(facePose);
    }

    return reefFaces;
  }

  /** Return all 12 reef scoring poses. */
  public static List<Pose2d> reefPoses() {
    return reefFaces().stream()
        .flatMap((face) -> faceToReefPair(face).stream())
        .collect(Collectors.toList());
  }

  /**
   * Given a pose looking at a face of the reef, calculate and return two poses parallel to the face
   * pose offset to the left and right.
   */
  public static List<Pose2d> faceToReefPair(Pose2d facePose) {
    return faceToReefPair(facePose, 0.15);
  }

  public static List<Pose2d> faceToReefPair(Pose2d facePose, double tangentLength) {
    var x = -Math.sin(facePose.getRotation().getRadians());
    var y = Math.cos(facePose.getRotation().getRadians());
    var length = Math.sqrt(x * x + y * y);
    var x2 = x / length * tangentLength;
    var y2 = y / length * tangentLength;

    // Allow scaling the tangent to the left reef and right reef independently
    // This is useful because the robot elevator is slightly off-center
    var leftScale = 1.1;
    var rightScale = 0.9;
    var leftPose =
        new Pose2d(
            facePose.getX() + x2 * leftScale,
            facePose.getY() + y2 * leftScale,
            facePose.getRotation());
    var rightPose =
        new Pose2d(
            facePose.getX() - x2 * rightScale,
            facePose.getY() - y2 * rightScale,
            facePose.getRotation());

    return List.of(leftPose, rightPose);
  }

  /** Returns the known poses of the april tags on the intake station. */
  public static List<Pose2d> intakeStationPoses() {
    return List.of(
        aprilTagLayout.getTagPose(12).get().toPose2d(),
        aprilTagLayout.getTagPose(13).get().toPose2d());
  }

  public static Pose2d offsetIntakePose(Pose2d stationPose) {
    return offsetPose(stationPose, -0.55, -0.5);
  }

  /**
   * Given an input pose, return a new pose that has the given offsets to the left and forward,
   * relative to the input pose.
   */
  public static Pose2d offsetPose(Pose2d inputPose, double offsetLeft, double offsetForward) {
    Rotation2d stationRotation = inputPose.getRotation();
    var backedOffOffsetPose =
        new Pose2d(
            offsetForward * stationRotation.plus(new Rotation2d(Math.PI)).getCos(),
            offsetForward * stationRotation.plus(new Rotation2d(Math.PI)).getSin(),
            new Rotation2d());
    var leftRightOffsetPose =
        new Pose2d(
            offsetLeft * stationRotation.plus(new Rotation2d(-Math.PI / 2)).getCos(),
            offsetLeft * stationRotation.plus(new Rotation2d(-Math.PI / 2)).getSin(),
            new Rotation2d());
    var derivedPose =
        new Pose2d(
            inputPose.getX() + backedOffOffsetPose.getX() + leftRightOffsetPose.getX(),
            inputPose.getY() + backedOffOffsetPose.getY() + leftRightOffsetPose.getY(),
            stationRotation.plus(new Rotation2d(Math.PI)));
    return derivedPose;
  }

  public static class ToClosestPose extends Command {
    private Drive drive;
    private boolean isLeft;
    private Command alignCommand;

    public ToClosestPose(Drive drive, boolean isLeft) {
      this.drive = drive;
      this.isLeft = isLeft;
    }

    @Override
    public void initialize() {

      var faces = AlignCommands.reefFaces();
      var reefPoses =
          faces.stream()
              .flatMap((face) -> AlignCommands.faceToReefPair(face).stream())
              .collect(Collectors.toList());
      var reefPosesArray = reefPoses.toArray(new Pose2d[reefPoses.size()]);
      Logger.recordOutput("RobotContainer/reefPoses", reefPosesArray);

      var reefLeftPoses =
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

      var reefRightPoses =
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

      List<Pose2d> currentPoses = null;

      if (DriverStation.isTeleop()) {

        if (isLeft) {
          currentPoses = reefLeftPoses;
        } else {
          currentPoses = reefRightPoses;
        }

      } else {
        currentPoses = reefPoses;
      }

      var drivePose = drive.getPose();

      var closestDistance = Double.MAX_VALUE;
      var closestPose = currentPoses.get(0);

      for (Pose2d pose : currentPoses) {
        var xDelta = Math.abs(drivePose.getX() - pose.getX());
        var yDelta = Math.abs(drivePose.getY() - pose.getY());
        var distance = Math.sqrt(xDelta * xDelta + yDelta * yDelta);

        if (distance < closestDistance) {
          closestDistance = distance;
          closestPose = pose;
        }
      }

      alignCommand = alignToPose(drive, closestPose);
      alignCommand.initialize();
    }

    @Override
    public void execute() {
      alignCommand.execute();
    }

    @Override
    public boolean isFinished() {
      return alignCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
      alignCommand.end(interrupted);
    }
  }
}
