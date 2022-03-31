package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/** Varies the shooter RPM based on range to target */
public class ShooterShootCommand extends CommandBase {

  // Instance of Shooter
  private final Shooter shooter;

  // Suppliers for hasTarget and targetRange
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetRangeSupplier;

  // Supplier of robotPose
  private final Supplier<Pose2d> robotPoseSupplier;

  // Position of the goal on the field
  private static final Pose2d GOAL_POSITION =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  /**
   * Constructs a ShooterAimCommand
   *
   * @param shooter An instance of Shooter
   */
  public ShooterShootCommand(
      Shooter shooter,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetRangeSupplier,
      Supplier<Pose2d> robotPoseSupplier) {
    // Set shooter
    this.shooter = shooter;

    // Set hasTarget and targetRange suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

    // Set robotPose supplier
    this.robotPoseSupplier = robotPoseSupplier;

    // Add the shooter as a requirement
    addRequirements(shooter);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if we have a target
    if (hasTargetSupplier.get()) {
      // If we have a vision target run the shooter RPM based off range
      shooter.shoot(targetRangeSupplier.get());
    } else {
      // If there is no target estimate the range to the target, run the shooter RPM based off that
      shooter.shoot(calculateRange());
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
}
