package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/**
 * This command runs the intake and feeder to intake a ball
 */
public class IntakeCommand extends ParallelCommandGroup {
  /**
   * Constructs an IntakeCommand
   * @param intake Instance of Intake
   * @param feeder Instance of Feeder
   */
  public IntakeCommand(Intake intake, Feeder feeder) {
    // Add commands to the ParallelCommandGroup
    addCommands(
        // StartEndCommand to start then stop the intake
        new StartEndCommand(intake::intake, intake::stop),
        // StartEndCommand to start then stop the feeder
        new StartEndCommand(feeder::intake, feeder::stop));

    // Add the intake and feeder as requirements
    addRequirements(intake, feeder);
  }
}
