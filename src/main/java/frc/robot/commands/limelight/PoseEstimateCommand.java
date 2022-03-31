package frc.robot.commands.limelight;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import java.util.function.Supplier;

/** Estimates robot pose based off of vision data */
public class PoseEstimateCommand extends CommandBase {

  // Instance of Limelight
  private final Limelight limelight;

  // Instance of SwerveDrivePoseEstimator
  private final SwerveDrivePoseEstimator poseEstimator;

  // Suppliers of turretAngle and gyroAngle
  private final Supplier<Double> turretAngleSupplier;
  private final Supplier<Double> gyroAngleSupplier;

  // Transforms
  private static final Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(90.0);
  private static final Pose2d GOAL_POSITION =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  /**
   * Constructs a PoseEstimateCommand
   *
   * @param limelight Instance of Limelight
   * @param poseEstimator Instance of SwerveDrivePoseEstimator
   * @param turretAngleSupplier Supplier of the current turret angle
   * @param gyroAngleSupplier Supplier of the current gyro angle
   */
  public PoseEstimateCommand(
      Limelight limelight,
      SwerveDrivePoseEstimator poseEstimator,
      Supplier<Double> turretAngleSupplier,
      Supplier<Double> gyroAngleSupplier) {
    // Set limelight
    this.limelight = limelight;

    // Set poseEstimator
    this.poseEstimator = poseEstimator;

    // Set suppliers
    this.turretAngleSupplier = turretAngleSupplier;
    this.gyroAngleSupplier = gyroAngleSupplier;

    // Add the limelight as a requirement
    addRequirements(limelight);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    if (limelight.getCurrentTarget().hasTarget) {
      // If we have a target calculate the robot pose based off the angle and range
      Pose2d robotPose =
          getRobotPose(
              limelight.getCurrentTarget().targetAngle, limelight.getCurrentTarget().range);

      // Add the vision measurement to the poseEstimator
      poseEstimator.addVisionMeasurement(robotPose, Timer.getFPGATimestamp());
    }
  }

  /**
   * Takes in an angle and range to the target and calculates the pose of the robot
   *
   * @param angle Angle to the target from the current turret angle
   * @param range Range to the target
   * @return The pose of the robot
   */
  private Pose2d getRobotPose(double angle, double range) {
    // Get the current turret angle
    Rotation2d turretAngle = Rotation2d.fromDegrees(turretAngleSupplier.get());

    // Rotate it by the angle to the target
    Rotation2d targetTurretAngle = turretAngle.rotateBy(Rotation2d.fromDegrees(angle));

    // Rotate that by TURRET_TO_ROBOT to make the angle robot relative
    Rotation2d targetRobotAngle = targetTurretAngle.rotateBy(TURRET_TO_ROBOT);

    // Rotate that by the current gyro angle to make the angle field relative
    Rotation2d targetFieldAngle =
        targetRobotAngle.rotateBy(Rotation2d.fromDegrees(gyroAngleSupplier.get()));

    // Get the position of the goal relative to the robot as a Transform2d
    Transform2d targetRobotTransform =
        new Transform2d(
            new Translation2d(
                -range * Math.cos(targetFieldAngle.getRadians()),
                -range * Math.sin(targetFieldAngle.getRadians())),
            new Rotation2d());

    // Return the position of the robot based off of the known position of the goal
    return GOAL_POSITION.transformBy(targetRobotTransform);
  }
}
