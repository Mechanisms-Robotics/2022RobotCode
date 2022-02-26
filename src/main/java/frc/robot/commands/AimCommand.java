package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** This command adjusts the shooter RPM, hood angle, and turret angle relative to a target. */
public class AimCommand extends CommandBase {
  // Subsystems
  private final Turret turret;
  private final Hood hood;

  // Suppliers of data from the GoalTracker
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetAngleSupplier;
  private final Supplier<Double> targetRangeSupplier;

  // Supplier of the boolean whether to go into fender shot mode
  private final Supplier<Boolean> fenderShotButton;

  // The position of the hood for the fender shot
  private static final double FENDER_HOOD_POSITION = -0.35;

  /**
   * Constructs an AimCommand
   *
   * @param turret Instance of Turret
   */
  public AimCommand(
      Turret turret,
      Hood hood,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetAngleSupplier,
      Supplier<Double> targetRangeSupplier,
      Supplier<Boolean> fenderShotButton) {
    this.turret = turret;
    this.hood = hood;

    this.hasTargetSupplier = hasTargetSupplier;
    this.targetAngleSupplier = targetAngleSupplier;
    this.targetRangeSupplier = targetRangeSupplier;

    this.fenderShotButton = fenderShotButton;

    // Add the shooter, hood, turret, and goalTracker as requirements
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if (!fenderShotButton.get()) {
      if (hasTargetSupplier.get()) {
        turret.aim(targetAngleSupplier.get());
        hood.aim(targetRangeSupplier.get());
      }
    } else {
      turret.goToZero();
      hood.setHoodRawPosition(FENDER_HOOD_POSITION);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }
}
