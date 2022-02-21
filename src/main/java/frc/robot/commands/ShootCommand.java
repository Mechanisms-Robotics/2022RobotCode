package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends ParallelCommandGroup {

  public ShootCommand(Shooter shooter, Accelerator accelerator, Feeder feeder, Hood hood) {
    addCommands(
        //new StartEndCommand(accelerator::shoot, accelerator::stop),
        new StartEndCommand(feeder::shoot, feeder::stop),
        new StartEndCommand(shooter::shoot, shooter::stop));

    addRequirements(shooter, feeder);
  }
}
