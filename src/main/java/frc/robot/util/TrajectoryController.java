package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** This class is used for controlling the swerve drive along a predefined trajectory. */
public class TrajectoryController {
  private static final double X_GAIN = 1.25 * 2.0;
  private static final double Y_GAIN = X_GAIN;
  private static final double THETA_GAIN = 2.0;
  private static final TrapezoidProfile.Constraints HEADING_PROFILE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(2 * Math.PI, 4 * Math.PI);

  private final Timer timer = new Timer();
  private PathPlannerTrajectory trajectory;
  private final SwerveDriveKinematics kinematics;
  private final HolonomicDriveController controller;
  private boolean isFinished = true;

  /**
   * Construction a Trajectory Controller
   *
   * @param kinematics The kinematics controller for the swerve drive.
   */
  public TrajectoryController(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;

    ProfiledPIDController thetaController =
        new ProfiledPIDController(THETA_GAIN, 0.0, 0.0, HEADING_PROFILE_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // Thanks mendax1234

    this.controller =
        new HolonomicDriveController(
            new PIDController(X_GAIN, 0.0, 0.0, Constants.loopTime),
            new PIDController(Y_GAIN, 0.0, 0.0, Constants.loopTime),
            thetaController);
  }

  /**
   * Start a trajectory. The calculate functions needs to be called periodically after this function
   * is called.
   *
   * @param trajectory The trajectory to follow.
   */
  public void startTrajectory(PathPlannerTrajectory trajectory) {
    isFinished = false;
    timer.reset();
    timer.start();
    this.trajectory = trajectory;
  }

  /**
   * Get the current trajectory
   *
   * @return The current trajectory
   */
  public PathPlannerTrajectory getTrajectory() {
    return trajectory;
  }

  /**
   * Call this function periodically after calling start trajectory. It returns the chassis speeds
   * needed to follow the trajectory.
   *
   * @param currentPose The current pose of the robot
   * @return The chassis speeds need to follow this trajectory.
   */
  public ChassisSpeeds calculate(Pose2d currentPose) {
    if (isFinished()) {
      return new ChassisSpeeds();
    }
    final double currentTime = timer.get();
    final var desiredState =
        (PathPlannerTrajectory.PathPlannerState) trajectory.sample(currentTime);
    desiredState.poseMeters = desiredState.poseMeters.transformBy(Constants.fieldRobot);
    desiredState.holonomicRotation.rotateBy(Constants.fieldRobot.getRotation());
    return controller.calculate(currentPose, desiredState, desiredState.holonomicRotation);
  }

  /**
   * Return weather the trajectory is finished or not. This is based on weather or not enough time
   * has passed not where the robot is.
   *
   * @return True if the trajectory is finish
   */
  public boolean isFinished() {
    if (trajectory == null) {
      return true;
    }
    if (timer.hasElapsed(trajectory.getTotalTimeSeconds())) {
      isFinished = true;
    }
    return isFinished;
  }

  /** Stop any currently running trajectory. */
  public void stop() {
    isFinished = true;
    timer.stop();
    timer.reset();
  }
}
