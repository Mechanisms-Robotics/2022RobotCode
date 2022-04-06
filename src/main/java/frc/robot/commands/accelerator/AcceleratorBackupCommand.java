package frc.robot.commands.accelerator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;

/** Runs the accelerator backwards to unjam a ball */
public class AcceleratorBackupCommand extends CommandBase {

  // Instance of Accelerator
  private final Accelerator accelerator;

  /**
   * Constructs a AcceleratorBackupCommand
   *
   * @param accelerator An instance of Accelerator
   */
  public AcceleratorBackupCommand(Accelerator accelerator) {
    // Set accelerator
    this.accelerator = accelerator;

    // Add the accelerator as a requirement
    addRequirements(accelerator);
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    // Backup the accelerator
    accelerator.backup();
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the accelerator
    accelerator.stop();
  }
}
