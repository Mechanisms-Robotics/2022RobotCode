package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/** Runs the shooter RPM based on range to target */
public class ShooterShootCommand extends CommandBase {

  // Instance of Shooter
  private final Shooter shooter;

  // Suppliers for hasTarget and targetRange
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetRangeSupplier;

  // Supplier for fender button
  private final Supplier<Boolean> fenderShotButtonSupplier;

  /**
   * Constructs a ShooterShootCommand
   *
   * @param shooter An instance of Shooter
   */
  public ShooterShootCommand(
      Shooter shooter, Supplier<Boolean> hasTargetSupplier, Supplier<Double> targetRangeSupplier, Supplier<Boolean> fenderShotButtonSupplier) {
    // Set shooter
    this.shooter = shooter;

    // Set hasTarget and targetRange suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

    // Set fender shot button supplier
    this.fenderShotButtonSupplier = fenderShotButtonSupplier;

    // Add the shooter as a requirement
    addRequirements(shooter);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if fender shot button is pressed
    if (fenderShotButtonSupplier.get()) {
      shooter.shoot(0.0);
      return;
    }

    // Check if we have a target
    if (hasTargetSupplier.get()) {
      // If we have a vision target run the shooter RPM based off range
      shooter.shoot(targetRangeSupplier.get());
    } else {
      // If there is no target idle the flywheels at the minimum RPM
      shooter.shoot(0.0);
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
