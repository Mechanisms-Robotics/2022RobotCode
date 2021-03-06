package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/** Runs the shooter backwards to unjam a ball */
public class ShooterBackupCommand extends CommandBase {

  // Instance of Shooter
  private final Shooter shooter;

  /**
   * Constructs a ShooterBackupCommand
   *
   * @param shooter An instance of Shooter
   */
  public ShooterBackupCommand(Shooter shooter) {
    // Set shooter
    this.shooter = shooter;

    // Add the shooter as a requirement
    addRequirements(shooter);
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    // Backup the shooter
    shooter.backup();
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
