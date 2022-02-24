package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/** This command runs the shooter, accelerator, and feeder to shoot a ball */
public class ShootCommand extends ParallelCommandGroup {
  /**
   * Constructs a ShootCommand
   *
   * @param shooter Instance of Shooter
   * @param accelerator Instance of Accelerator
   * @param feeder Instance of Feeder
   */
  public ShootCommand(Shooter shooter, Accelerator accelerator, Feeder feeder) {
    // Add commands to the ParallelCommandGroup
    addCommands(
        // StartEndCommand to run the accelerator then stop it
        new StartEndCommand(accelerator::shoot, accelerator::stop),
        // StartEndCommand to run the feeder then stop it
        new StartEndCommand(feeder::shoot, feeder::stop),
        // StartEndCommand to run the shooter then stop it
        new StartEndCommand(shooter::shoot, shooter::stop));

    // Add the shooter, accelerator, and feeder as a requirement
    addRequirements(shooter, accelerator, feeder);
  }
}
