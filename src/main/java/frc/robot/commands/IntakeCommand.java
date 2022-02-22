package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends ParallelCommandGroup {
  public IntakeCommand(Intake intake, Feeder feeder, Accelerator accelerator) {
    addCommands(
        new StartEndCommand(intake::intake, intake::stop),
        new StartEndCommand(feeder::intake, feeder::stop),
        new StartEndCommand(accelerator::idle, accelerator::stop));

    addRequirements(intake, feeder);
  }
}