package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** Aims the turret at the goal */
public class TurretAimCommand extends CommandBase {

  // Instance of Turret
  private final Turret turret;

  // Suppliers for hasTarget, targetAngle, and targetRange
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetAngleSupplier;
  private final Supplier<Double> targetRangeSupplier;

  // Supplier of robotPose and robotVelocity
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;

  // Transforms
  private static final Pose2d GOAL_POSITION =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());
  private static final Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(-90.0);

  /**
   * Constructs a TurretAimCommand
   *
   * @param turret An instance of Turret
   */
  public TurretAimCommand(
      Turret turret,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetAngleSupplier,
      Supplier<Double> targetRangeSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    // Set turret
    this.turret = turret;

    // Set hasTarget, targetAngle, and targetRange suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetAngleSupplier = targetAngleSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

    // Set robotPose and robotVelocity supplier
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;

    // Add the turret as a requirement
    addRequirements(turret);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if we have a target
    if (hasTargetSupplier.get()) {
      // If we have a vision target aim the turret at it compensating for robot movement
      turret.aim(
          calculateMovingGoalAngle(
              new Rotation2d(targetAngleSupplier.get()),
              targetRangeSupplier.get(),
              new Rotation2d(turret.getAngle()),
              robotPoseSupplier.get(),
              robotVelocitySupplier.get()));
    } else {
      // If we don't estimate the goal position and aim at it compensating for robot movement
      turret.aim(
          calculateMovingGoalAngle(
              new Rotation2d(turret.getAngle()),
              robotPoseSupplier.get(),
              robotVelocitySupplier.get()));
    }
  }

  /**
   * Calculates the angle to aim at a target while moving
   *
   * @param angle The angle to the target from the current turret angle
   * @param range The range to the target
   * @param currentTurretAngle The current turret angle
   * @param robotPose The estimated robot pose
   * @param velocity The robot velocity
   * @return Angle to aim at
   */
  public static double calculateMovingGoalAngle(
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

    // Get an angle to the goal trail relative to the field
    fieldAngle = new Rotation2d(trailPose.getX(), trailPose.getY());

    // Rotate that by TURRET_TO_ROBOT and the gyro angle to get an angle relative to the turret
    turretAngle = currentTurretAngle.rotateBy(TURRET_TO_ROBOT).rotateBy(robotPose.getRotation());

    // Subtract the angle relative to the turret from the field angle to get an angle offset
    Rotation2d angleOffset = fieldAngle.minus(turretAngle);

    // Return the angle offset
    return angleOffset.getDegrees();
  }

  /**
   * Calculates the angle to aim at a target while moving
   *
   * @param currentTurretAngle The current turret angle
   * @param robotPose The estimated robot pose
   * @param velocity The robot velocity
   * @return Angle to aim at
   */
  public static double calculateMovingGoalAngle(
      Rotation2d currentTurretAngle, Pose2d robotPose, ChassisSpeeds velocity) {
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

    // Get an angle to the goal trail relative to the field
    final Rotation2d fieldAngle = new Rotation2d(trailPose.getX(), trailPose.getY());

    // Rotate that by TURRET_TO_ROBOT and the gyro angle to get an angle relative to the turret
    final Rotation2d turretAngle =
        currentTurretAngle.rotateBy(TURRET_TO_ROBOT).rotateBy(robotPose.getRotation());

    // Subtract the angle relative to the turret from the field angle to get an angle offset
    Rotation2d angleOffset = fieldAngle.minus(turretAngle);

    // Return the angle offset
    return angleOffset.getDegrees();
  }
}
