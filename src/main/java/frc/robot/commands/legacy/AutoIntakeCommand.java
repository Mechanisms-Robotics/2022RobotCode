package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.SetIntakeCommand.IntakeMode;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/** This command runs the intake and feeder to intake a ball */
public class AutoIntakeCommand extends ParallelCommandGroup {
  /**
   * Constructs an IntakeCommand
   *
   * @param intake Instance of Intake
   * @param feeder Instance of Feeder
   */
  public AutoIntakeCommand(Intake intake, Feeder feeder, Accelerator accelerator) {
    // Add commands to the ParallelCommandGroup
    addCommands(
        new SetIntakeCommand(intake, IntakeMode.DEPLOY),
        // StartEndCommand to start then stop the intake
        new StartEndCommand(intake::intake, intake::stop),
        // StartEndCommand to start then stop the feeder
        new PrepFeederCommand(feeder, accelerator).andThen(new BackupCommand(accelerator, feeder)));
    addRequirements(intake);
  }
}
