package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** This command adjusts the shooter RPM, hood angle, and turret angle relative to a target. */
public class AimCommand extends CommandBase {
  // Subsystems
  private final Turret turret;
  private final Hood hood;
  private final Limelight limelight;

  // Supplier of the boolean whether to go into fender shot mode
  private final Supplier<Boolean> fenderShotButton;

  // The position of the hood for the fender shot
  private static final double FENDER_HOOD_POSITION = -0.25;

  /**
   * Constructs an AimCommand
   *
   * @param turret Instance of Turret
   */
  public AimCommand(
      Turret turret,
      Hood hood,
      Limelight limelight,
      Supplier<Boolean> fenderShotButton) {
    this.turret = turret;
    this.hood = hood;

    this.limelight = limelight;

    this.fenderShotButton = fenderShotButton;

    // Add the shooter, hood, turret, and goalTracker as requirements
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if (!fenderShotButton.get()) {
      var target = limelight.getCurrentTarget();
      if (target.hasTarget) {
        turret.aim(target.targetAngle);
        hood.aim(target.range);
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
