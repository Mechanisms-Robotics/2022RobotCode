package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

/** Runs the feeder backwards to unjam a ball */
public class FeederBackupCommand extends CommandBase {

  // Instance of Feeder
  private final Feeder feeder;

  /**
   * Constructs a FeederBackupCommand
   *
   * @param feeder An instance of Feeder
   */
  public FeederBackupCommand(Feeder feeder) {
    // Set feeder
    this.feeder = feeder;

    // Add the feeder as a requirement
    addRequirements(feeder);
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    // Backup the feeder
    feeder.backup();
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the feeder
    feeder.stop();
  }
}
