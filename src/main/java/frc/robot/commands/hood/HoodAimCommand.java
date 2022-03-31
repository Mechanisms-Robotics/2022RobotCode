package frc.robot.commands.hood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import java.util.function.Supplier;

// Aims the hood based on a range to a target
public class HoodAimCommand extends CommandBase {

  // Instance of Hood
  private final Hood hood;

  // Suppliers of hasTarget and targetRange
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetRangeSupplier;

  // Supplier of robotPose
  private final Supplier<Pose2d> robotPoseSupplier;

  // Position of the goal on the field
  private static final Pose2d GOAL_POSITION =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  /**
   * Constructs a HoodAimCommand
   *
   * @param hood Instance of Hood
   */
  public HoodAimCommand(
      Hood hood, Supplier<Boolean> hasTargetSupplier, Supplier<Double> targetRangeSupplier, Supplier<Pose2d> robotPoseSupplier) {
    // Set hood
    this.hood = hood;

    // Set hasTarget and targetRange suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

    // Set robotPose supplier
    this.robotPoseSupplier = robotPoseSupplier;

    // Add the hood as a requirement
    addRequirements(hood);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    if (hasTargetSupplier.get()) {
      // If we have a target aim the hood at it
      hood.aim(targetRangeSupplier.get());
    } else {
      // If we don't have a target estimate a range to the goal and aim the hood
      hood.aim(calculateRange());
    }
  }

  /**
   * Calculates the range to the goal based off of an estimated robot pose
   *
   * @return Estimated range to the goal
   */
  private double calculateRange() {
    return GOAL_POSITION.minus(robotPoseSupplier.get()).getTranslation().getNorm();
  }
}
