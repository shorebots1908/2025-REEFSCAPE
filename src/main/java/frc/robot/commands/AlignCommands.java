package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

/**
 * AlignCommands contains static functions for constructing a Command that will drive the Robot to a
 * Pose2d using three profiled PID controllers.
 *
 * <p>The overloaded functions allow us to set defaults for various inputs needed by the actual
 * command, which is constructed in the last function at the bottom. To override defaults, just call
 * the function that lets you pick all the inputs.
 */
public class AlignCommands {
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

  /**
   * Given a Drive subsystem and a target Pose2d, this command will use three profiled PID
   * controllers to drive the robot to the pose.
   */
  public static Command alignToPose(Drive drive, Pose2d targetPose) {
    // AlignConfig is P, I, D, Velocity, Acceleration
    var alignConfig = new AlignConfig(1, 0, 0, 1, 1);
    return alignToPose(drive, targetPose, alignConfig);
  }

  /**
   * Provide an AlignConfig with custom gains for P, I, D, and contraints for Velocity and
   * Acceleration, and get a Command that drives with those characteristics.
   */
  public static Command alignToPose(Drive drive, Pose2d targetPose, AlignConfig config) {
    var xPid =
        new ProfiledPIDController(
            config.p,
            config.i,
            config.d,
            new TrapezoidProfile.Constraints(config.velocity, config.acceleration));
    var yPid =
        new ProfiledPIDController(
            config.p,
            config.i,
            config.d,
            new TrapezoidProfile.Constraints(config.velocity, config.acceleration));
    var rPid =
        new ProfiledPIDController(
            config.p,
            config.i,
            config.d,
            new TrapezoidProfile.Constraints(config.velocity, config.acceleration));

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
    var initialPose = drive.getPose();
    xPid.reset(initialPose.getX());
    yPid.reset(initialPose.getY());
    rPid.reset(initialPose.getRotation().getRadians());

    return Commands.run(
        () -> {
          var drivePose = drive.getPose();
          var xErr = Math.abs(drivePose.getX() - targetPose.getX());
          var yErr = Math.abs(drivePose.getY() - targetPose.getY());
          var rErr =
              Math.abs(
                  drivePose.getRotation().getRadians() - targetPose.getRotation().getRadians());
          var xOut = xPid.calculate(xErr);
          var yOut = yPid.calculate(yErr);
          var rOut = rPid.calculate(rErr);
          var fieldSpeeds = new ChassisSpeeds(xOut, yOut, rOut);
          var driveSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, drivePose.getRotation());
          drive.runVelocity(driveSpeeds);
        },
        drive);
  }

  public static double TAU = Math.PI * 2;

  /** Generate the reef face poses using the default geometry. */
  public static List<Pose2d> reefFaces() {
    // Geometry setup: Need a center point and radius for the reef circle
    var reefCenter = new Pose2d(4.4895, 4.026, Rotation2d.fromRadians(0.0));
    var reefRadius = 0.8315 + 0.44;
    return reefFaces(reefCenter, reefRadius);
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

  public static List<Pose2d> faceToReefPair(Pose2d facePose) {
    return faceToReefPair(facePose, 0.15);
  }

  public static List<Pose2d> faceToReefPair(Pose2d facePose, double tangentLength) {
    var x = -Math.sin(facePose.getRotation().getRadians());
    var y = Math.cos(facePose.getRotation().getRadians());
    var length = Math.sqrt(x * x + y * y);
    var x2 = x / length * tangentLength;
    var y2 = y / length * tangentLength;
    var pose1 = new Pose2d(facePose.getX() + x2, facePose.getY() + y2, facePose.getRotation());
    var pose2 = new Pose2d(facePose.getX() - x2, facePose.getY() - y2, facePose.getRotation());

    return List.of(pose1, pose2);
  }
}
