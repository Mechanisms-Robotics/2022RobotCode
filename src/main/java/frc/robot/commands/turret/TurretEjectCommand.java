package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** Aims the turret away from the goal */
public class TurretEjectCommand extends CommandBase {

  // Instance of Turret
  private final Turret turret;

  // Suppliers for hasTarget and targetAngle
  private Supplier<Boolean> hasTargetSupplier;
  private Supplier<Double> targetAngleSupplier;

  // How much to aim away from the target
  private static final double EJECT_OFFSET = 45.0; // degrees

  /**
   * Constructs a TurretEjectCommand
   *
   * @param turret An instance of Turret
   */
  public TurretEjectCommand(
      Turret turret, Supplier<Boolean> hasTargetSupplier, Supplier<Double> targetAngleSupplier) {
    // Set turret
    this.turret = turret;

    // Set hasTarget and targetAngle suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetAngleSupplier = targetAngleSupplier;

    // Add the turret as a requirement
    addRequirements(turret);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if we have a target
    if (hasTargetSupplier.get()) {
      // If we have a vision target aim the turret away from it by EJECT_OFFSET
      turret.aim(targetAngleSupplier.get() + EJECT_OFFSET);
    }
  }
}
