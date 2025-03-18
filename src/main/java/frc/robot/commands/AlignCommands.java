package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

/**
 * AlignCommands contains static functions for constructing a Command that will drive
 * the Robot to a Pose2d using three profiled PID controllers.
 * 
 * The overloaded functions allow us to set defaults for various inputs needed by the
 * actual command, which is constructed in the last function at the bottom. To override
 * defaults, just call the function that lets you pick all the inputs.
 */
public class AlignCommands {
  /**
   * The ProfiledPIDControllers we use need inputs for P, I, D, Velocity, and Acceleration
   */
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
   * Given a Drive subsystem and a target Pose2d, this command will use
   * three profiled PID controllers to drive the robot to the pose.
   */
  public static Command alignToPose(Drive drive, Pose2d targetPose) {
    // AlignConfig is P, I, D, Velocity, Acceleration
    var alignConfig = new AlignConfig(1, 0, 0, 1, 1);
    return alignToPose(drive, targetPose, alignConfig);
  }

  /**
   * Provide an AlignConfig with custom gains for P, I, D, and contraints for
   * Velocity and Acceleration, and get a Command that drives with those characteristics.
   */
  public static Command alignToPose(Drive drive, Pose2d targetPose, AlignConfig config) {
    var xPid = new ProfiledPIDController(config.p, config.i, config.d,
      new TrapezoidProfile.Constraints(config.velocity, config.acceleration));
    var yPid = new ProfiledPIDController(config.p, config.i, config.d,
      new TrapezoidProfile.Constraints(config.velocity, config.acceleration));
    var rPid = new ProfiledPIDController(config.p, config.i, config.d,
      new TrapezoidProfile.Constraints(config.velocity, config.acceleration));

    return alignToPose(drive, targetPose, xPid, yPid, rPid);
  }

  /**
   * Provide the actual ProfiledPIDControllers to use for alignment, and get a
   * Command that drives using those controllers.
   */
  public static Command alignToPose(
    Drive drive, 
    Pose2d targetPose,
    ProfiledPIDController xPid,
    ProfiledPIDController yPid,
    ProfiledPIDController rPid
  ) {
    var initialPose = drive.getPose();
    xPid.reset(initialPose.getX());
    yPid.reset(initialPose.getY());
    rPid.reset(initialPose.getRotation().getRadians());

    return Commands.run(() -> {
      var drivePose = drive.getPose();
      var xErr = Math.abs(drivePose.getX() - targetPose.getX());
      var yErr = Math.abs(drivePose.getY() - targetPose.getY());
      var rErr = Math.abs(
        drivePose.getRotation().getRadians()
        - targetPose.getRotation().getRadians()
      );
      var xOut = xPid.calculate(xErr);
      var yOut = xPid.calculate(yErr);
      var rOut = xPid.calculate(rErr);
      var fieldSpeeds = new ChassisSpeeds(xOut, yOut, rOut);
      var driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, drivePose.getRotation());
      drive.runVelocity(driveSpeeds);
    }, drive);
  }
}
