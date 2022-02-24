package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/** This command will outtake the intake, feeder, and accelerator */
public class OuttakeCommand extends ParallelCommandGroup {
  /**
   * Constructs an OuttakeCommand
   *
   * @param intake Instance of Intake
   * @param feeder Instance of Feeder
   * @param accelerator Instance of Accelerator
   */
  public OuttakeCommand(Intake intake, Feeder feeder, Accelerator accelerator) {
    // Add commands to the ParallelCommandGroup
    addCommands(
        // StartEndCommand to outtake then stop the intake
        new StartEndCommand(intake::outtake, intake::stop),
        // StartEndCommand to outtake then stop the feeder
        new StartEndCommand(feeder::outtake, feeder::stop),
        // StartEndCommand to outtake then stop the accelerator
        new StartEndCommand(accelerator::outtake, accelerator::stop));

    // Add the intake, feeder, and accelerator as requirements
    addRequirements(intake, feeder, accelerator);
  }
}
