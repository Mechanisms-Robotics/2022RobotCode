package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;

/**
 * This command backs up the accelerator and feeder for a set amount of time
 */
public class BackupCommand extends ParallelCommandGroup {
  // Time to back it up for
  private static final double BACKUP_TIME = 1.0; // Seconds

  /**
   * Constructs a BackupCommand
   * @param accelerator An instance of Accelerator
   * @param feeder An instance of Feeder
   */
  public BackupCommand(Accelerator accelerator, Feeder feeder) {
    // Add commands to the ParallelCommandGroup
    addCommands(
        // A start end command to back up then stop the accelerator and feeder with a timeout of BACKUP_TIME
        new StartEndCommand(
                () -> {
                  accelerator.backup();
                  feeder.backup();
                },
                () -> {
                  accelerator.stop();
                  feeder.stop();
                })
            .withTimeout(BACKUP_TIME));

    // Add the accelerator and feeder as a requirement
    addRequirements(accelerator, feeder);
  }
}
