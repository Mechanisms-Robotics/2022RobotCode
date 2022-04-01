package frc.robot.commands.shooter;

import static frc.robot.commands.shooter.ShooterAimCommand.calculateMovingGoalRange;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/** Varies the shooter RPM based on range to target */
public class ShooterShootCommand extends CommandBase {

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

  /**
   * Constructs a ShooterShootCommand
   *
   * @param shooter Instance of shooter
   * @param hasTargetSupplier Supplier of hasTarget
   * @param targetAngleSupplier Supplier of targetAngle
   * @param targetRangeSupplier Supplier of targetRange
   * @param robotPoseSupplier Supplier of robotPose
   * @param robotVelocitySupplier Supplier of robotVelocity
   */
  public ShooterShootCommand(
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
}
