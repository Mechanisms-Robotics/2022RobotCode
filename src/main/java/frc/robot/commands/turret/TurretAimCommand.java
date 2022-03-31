package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** Aims the turret at the goal */
public class TurretAimCommand extends CommandBase {

  // Instance of Turret
  private final Turret turret;

  // Suppliers for hasTarget and targetAngle
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetAngleSupplier;

  // Supplier of robotPose
  private final Supplier<Pose2d> robotPoseSupplier;

  // Transforms
  private static final Pose2d GOAL_POSITION =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());
  private static final Rotation2d ROBOT_TO_TURRET = Rotation2d.fromDegrees(-90.0);

  /**
   * Constructs a TurretAimCommand
   *
   * @param turret An instance of Turret
   */
  public TurretAimCommand(
      Turret turret,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetAngleSupplier,
      Supplier<Pose2d> robotPoseSupplier) {
    // Set turret
    this.turret = turret;

    // Set hasTarget and targetAngle suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetAngleSupplier = targetAngleSupplier;

    // Set robotPose supplier
    this.robotPoseSupplier = robotPoseSupplier;

    // Add the turret as a requirement
    addRequirements(turret);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if we have a target
    if (hasTargetSupplier.get()) {
      // If we have a vision target aim the turret at it
      turret.aim(targetAngleSupplier.get());
    } else {
      // If we don't have a vision target calculate the angle to where the goal should be and aim
      turret.aim(calculateGoalAngle(robotPoseSupplier.get(), new Rotation2d(turret.getAngle())));
    }
  }

  /**
   * Calculates turret angle to goal based off of an estimated robot pose
   *
   * @return Turret angle to goal
   */
  public static double calculateGoalAngle(Pose2d robotPose, Rotation2d currentTurretAngle) {
    // Unit vector and vector to goal
    Translation2d unitVec = new Translation2d(1.0, 0.0);
    Translation2d goalVec =
        GOAL_POSITION
            .transformBy(new Transform2d(robotPose.getTranslation().unaryMinus(), new Rotation2d()))
            .getTranslation();

    // Calculate the angle between them
    Rotation2d fieldAngle =
        new Rotation2d(
            Math.acos(
                ((unitVec.getX() * goalVec.getX()) + (unitVec.getY() * goalVec.getY()))
                    / (unitVec.getNorm() * goalVec.getNorm())));

    // Rotate that so the angle is relative to the robot
    Rotation2d robotAngle = robotPose.getRotation().rotateBy(fieldAngle);

    // Rotate that so the angle is relative to the turret
    Rotation2d turretAngle = robotAngle.rotateBy(ROBOT_TO_TURRET);

    // Rotate that so the angle is an offset from the current turret angle
    Rotation2d angleOffset = turretAngle.rotateBy(currentTurretAngle);

    // Wrap the angle depending on which quadrant we are in
    angleOffset =
        wrapAngle(
            angleOffset,
            robotPose.getY() > GOAL_POSITION.getY(),
            robotPose.getX() > GOAL_POSITION.getX());

    // Return the angle offset
    return angleOffset.getDegrees();
  }

  /**
   * Wraps angle depending on which quadrant it is in
   *
   * @param angle The angle to wrap
   * @param above Whether we are above the origin
   * @param right Whether we are to the right of the origin
   * @return The wrapped angle
   */
  private static Rotation2d wrapAngle(Rotation2d angle, boolean above, boolean right) {
    if (above && !right) {
      // If we are in the top left quadrant invert the angle
      return angle.unaryMinus();
    }
    if (right && !above) {
      // If we are in the bottom right subtract the angle from 180
      return Rotation2d.fromDegrees(180.0).minus(angle);
    }

    // If we are in the bottom left or top right quadrant just return the angle
    return angle;
  }
}
