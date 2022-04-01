package frc.robot.commands.hood;

import static frc.robot.commands.shooter.ShooterAimCommand.calculateMovingGoalRange;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import java.util.function.Supplier;

// Aims the hood based on a range to a target
public class HoodAimCommand extends CommandBase {

  // Instance of Hood
  private final Hood hood;

  // Suppliers for hasTarget, targetAngle, and targetRange
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetAngleSupplier;
  private final Supplier<Double> targetRangeSupplier;

  // Supplier of currentTurretAngle, robotPose, and robotVelocity
  private final Supplier<Double> currentTurretAngleSupplier;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;

  /**
   * Constructs a HoodAimCommand
   *
   * @param hood Instance of Hood
   */
  public HoodAimCommand(
      Hood hood,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetAngleSupplier,
      Supplier<Double> targetRangeSupplier,
      Supplier<Double> currentTurretAngleSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    // Set hood
    this.hood = hood;

    // Set hasTarget, targetAngle, and targetRange suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetAngleSupplier = targetAngleSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

    // Set currentTurretAngle, robotPose, and robotVelocity
    this.currentTurretAngleSupplier = currentTurretAngleSupplier;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;

    // Add the hood as a requirement
    addRequirements(hood);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    if (hasTargetSupplier.get()) {
      // If we have a target aim the hood at it accounting for movement
      hood.aim(
          calculateMovingGoalRange(
              Rotation2d.fromDegrees(targetAngleSupplier.get()),
              targetRangeSupplier.get(),
              new Rotation2d(currentTurretAngleSupplier.get()),
              robotPoseSupplier.get(),
              robotVelocitySupplier.get()));
    } else {
      // If we don't have a target estimate a range to the goal and account for movement
      hood.aim(
          calculateMovingGoalRange(
              robotPoseSupplier.get(),
              robotVelocitySupplier.get()));
    }
  }
}
