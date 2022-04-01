package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/** Varies the shooter RPM based on range to target */
public class ShooterAimCommand extends CommandBase {

  // Instance of Shooter
  private final Shooter shooter;

  // Suppliers for hasTarget, targetAngle, and targetRange
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetAngleSupplier;
  private final Supplier<Double> targetRangeSupplier;

  // Supplier of currentTurretAngle, robotPose, and robotVelocity
  private final Supplier<Double> currentTurretAngleSupplier;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;

  // Transforms
  private static final Pose2d GOAL_POSITION =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());
  private static final Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(-90.0);

  /**
   * Constructs a ShooterAimCommand
   *
   * @param shooter Instance of shooter
   * @param hasTargetSupplier Supplier of hasTarget
   * @param targetAngleSupplier Supplier of targetAngle
   * @param targetRangeSupplier Supplier of targetRange
   * @param robotPoseSupplier Supplier of robotPose
   * @param robotVelocitySupplier Supplier of robotVelocity
   */
  public ShooterAimCommand(
      Shooter shooter,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetAngleSupplier,
      Supplier<Double> targetRangeSupplier,
      Supplier<Double> currentTurretAngleSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    // Set shooter
    this.shooter = shooter;

    // Set hasTarget, targetAngle, and targetRange suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetAngleSupplier = targetAngleSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

    // Set currentTurretAngle, robotPose, and robotVelocity
    this.currentTurretAngleSupplier = currentTurretAngleSupplier;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;

    // Add the shooter as a requirement
    addRequirements(shooter);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if we have a target
    if (hasTargetSupplier.get()) {
      // If we have a vision target run the shooter RPM based off range accounting for movement
      shooter.shoot(
          calculateMovingGoalRange(
              Rotation2d.fromDegrees(targetAngleSupplier.get()),
              targetRangeSupplier.get(),
              new Rotation2d(currentTurretAngleSupplier.get()),
              robotPoseSupplier.get(),
              robotVelocitySupplier.get()));
    } else {
      // If there is no target estimate the range to the target, and account for movement
      shooter.shoot(
          calculateMovingGoalRange(
              robotPoseSupplier.get(),
              robotVelocitySupplier.get()));
    }
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the shooter
    shooter.stop();
  }

  /**
   * Calculates the range to the goal based off of an estimated robot pose
   *
   * @return Estimated range to the goal
   */
  private double calculateRange() {
    return GOAL_POSITION.minus(robotPoseSupplier.get()).getTranslation().getNorm();
  }

  /**
   * Calculates the range to the goal while moving
   *
   * @param angle The angle to the target from the current turret angle
   * @param range The range to the target
   * @param currentTurretAngle The current turret angle
   * @param robotPose The estimated robot pose
   * @param velocity The robot velocity
   * @return Range to the goal trail
   */
  public static double calculateMovingGoalRange(
      Rotation2d angle,
      double range,
      Rotation2d currentTurretAngle,
      Pose2d robotPose,
      ChassisSpeeds velocity) {
    // Rotate the angle by the current turret angle to get an absolute turret angle
    Rotation2d turretAngle = angle.rotateBy(currentTurretAngle);

    // Rotate that by TURRET_TO_ROBOT to get an angle relative to the robot
    Rotation2d robotAngle = turretAngle.rotateBy(TURRET_TO_ROBOT);

    // Rotate that by the current gyro angle to get an angle relative to the field
    Rotation2d fieldAngle = robotAngle.rotateBy(robotPose.getRotation());

    // Subtract 360 from that to get the angle between the field forward and the goal
    final Rotation2d fieldToGoalAngle = fieldAngle.minus(new Rotation2d(2 * Math.PI));

    // Calculate the goal pose
    final Translation2d goalPose =
        new Translation2d(
            range * Math.cos(fieldToGoalAngle.getRadians()),
            range * Math.sin(fieldToGoalAngle.getRadians()));

    // Calculate the robot's velocity vector
    final Translation2d velocityVec =
        new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);

    // TODO: Calculate dynamically
    final double airTime = 1.25; // seconds

    // Transform the goal by the inverse of the robot's velocity multiplied by the ball's air time
    final Translation2d trailPose = goalPose.minus(velocityVec.times(airTime));

    // Return the range to the goal trail
    return trailPose.getNorm();
  }

  /**
   * Calculates the range to the goal while moving
   *
   * @param robotPose The estimated robot pose
   * @param velocity The robot velocity
   * @return The range to the goal trail
   */
  public static double calculateMovingGoalRange(Pose2d robotPose, ChassisSpeeds velocity) {
    // Calculate the goal pose relative to the robot
    final Translation2d goalPose =
        GOAL_POSITION
            .transformBy(new Transform2d(robotPose.getTranslation().unaryMinus(), new Rotation2d()))
            .getTranslation();

    // Calculate the robot's velocity vector
    final Translation2d velocityVec =
        new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);


    // TODO: Calculate dynamically
    final double airTime = 1.25; // seconds

    // Transform the goal by the inverse of the robot's velocity multiplied by the ball's air time
    final Translation2d trailPose = goalPose.minus(velocityVec.times(airTime));

    // Return the range to the goal trail
    return trailPose.getNorm();
  }
}
