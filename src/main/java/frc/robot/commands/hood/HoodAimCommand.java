package frc.robot.commands.hood;

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

  // Supplier for fender button
  private final Supplier<Boolean> fenderShotButtonSupplier;

  /**
   * Constructs a HoodAimCommand
   *
   * @param hood Instance of Hood
   */
  public HoodAimCommand(
      Hood hood, Supplier<Boolean> hasTargetSupplier, Supplier<Double> targetRangeSupplier, Supplier<Boolean> fenderShotButtonSupplier) {
    // Set hood
    this.hood = hood;

    // Set hasTarget and targetRange suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

    // Set fender shot button supplier
    this.fenderShotButtonSupplier = fenderShotButtonSupplier;

    // Add the hood as a requirement
    addRequirements(hood);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if fender shot button is pressed
    if (fenderShotButtonSupplier.get()) {
      hood.aim(0.0);
      return;
    }

    if (hasTargetSupplier.get()) {
      // If we have a target aim the hood at it
      hood.aim(targetRangeSupplier.get());
    } else {
      // If we don't have a target set the hood to fender shot position
      hood.aim(0.0);
    }
  }
}
