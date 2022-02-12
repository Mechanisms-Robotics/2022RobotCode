package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;

public class BackupCommand extends ParallelCommandGroup {
  private static final double BACKUP_TIME = 0.25; // Seconds

  public BackupCommand(Accelerator accelerator, Feeder feeder) {
    addCommands(
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

    addRequirements(accelerator, feeder);
  }
}
