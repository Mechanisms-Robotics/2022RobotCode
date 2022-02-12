package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class OuttakeCommand extends ParallelCommandGroup {
  public OuttakeCommand(Intake intake, Feeder feeder, Accelerator accelerator) {
    addCommands(
        new StartEndCommand(intake::outtake, intake::stop),
        new StartEndCommand(feeder::outtake, feeder::stop),
        new StartEndCommand(accelerator::outtake, accelerator::stop));

    addRequirements(intake, feeder, accelerator);
  }
}
