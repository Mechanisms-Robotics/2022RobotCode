package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** Aims the turret at the goal */
public class TurretAimCommand extends CommandBase {

  // Instance of Turret
  private final Turret turret;

  // Suppliers for hasTarget and targetAngle
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Double> targetAngleSupplier;

  // Supplier for fender button
  private final Supplier<Boolean> fenderShotButtonSupplier;

  // Whether we had a target or not
  private final Timer lastSeenTimer = new Timer();
  private static final double SNAP_TIME = 1.5;
  private boolean hadTarget = false;

  /**
   * Constructs a TurretAimCommand
   *
   * @param turret An instance of Turret
   */
  public TurretAimCommand(
      Turret turret, Supplier<Boolean> hasTargetSupplier, Supplier<Double> targetAngleSupplier, Supplier<Boolean> fenderShotButtonSupplier) {
    // Set turret
    this.turret = turret;

    // Set hasTarget and targetAngle suppliers
    this.hasTargetSupplier = hasTargetSupplier;
    this.targetAngleSupplier = targetAngleSupplier;

    // Set fender shot button supplier
    this.fenderShotButtonSupplier = fenderShotButtonSupplier;

    // Add the turret as a requirement
    addRequirements(turret);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if fender shot button is pressed
    if (fenderShotButtonSupplier.get()) {
      turret.snapTo(0.0);
      return;
    }

    // Check if we have a target
    if (hasTargetSupplier.get()) {
      // If we have a vision target aim the turret at it
      turret.aim(targetAngleSupplier.get());

      // Set hadTarget to true
      hadTarget = true;
    } else if (hadTarget) {
      lastSeenTimer.reset();
      lastSeenTimer.start();

      if (Math.abs(turret.getAngle() - Turret.TURRET_FORWARD_LIMIT) > Math.abs(turret.getAngle() - Turret.TURRET_REVERSE_LIMIT)) {
        turret.snapDirection = false;
      } else {
        turret.snapDirection = true;
      }

      hadTarget = false;
      //        hood.aim(this.lastTargetRange);
      //        turret.aim(this.targetAngle.getDegrees(), swerve.getSpeeds(),
      // this.lastTargetRange);
    } else if (lastSeenTimer.hasElapsed(SNAP_TIME)) {
      turret.snapDirection = !turret.snapDirection;
      turret.snapAround();

      lastSeenTimer.stop();
      lastSeenTimer.reset();
      lastSeenTimer.start();
    }
  }
}
