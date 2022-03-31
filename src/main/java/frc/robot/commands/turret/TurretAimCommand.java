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
  public static double calculateGoalAngle(
      final Pose2d robotPose, final Rotation2d currentTurretAngle) {
    // Vector from the robot to the goal
    final Translation2d robotToGoal =
        GOAL_POSITION
            .transformBy(new Transform2d(robotPose.getTranslation().unaryMinus(), new Rotation2d()))
            .getTranslation();

    // Angle of the goal relative to the field
    final Rotation2d fieldAngle = new Rotation2d(robotToGoal.getX(), robotToGoal.getY());

    // Angle of the goal relative to the turret zero
    final Rotation2d turretAngle =
        currentTurretAngle.rotateBy(TURRET_TO_ROBOT).rotateBy(robotPose.getRotation());

    // Angle of the goal as an offset from the current turret angle
    Rotation2d angleOffset = fieldAngle.minus(turretAngle);

    // Return the angle offset
    return angleOffset.getDegrees();
  }
}
