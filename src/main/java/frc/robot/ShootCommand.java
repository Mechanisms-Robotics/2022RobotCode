package frc.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand(Feeder feeder, Shooter shooter, Accelerator accelerator) {
        addCommands(new StartEndCommand(feeder::feed, feeder::stop),
            new StartEndCommand(shooter::shoot, shooter::stop),
            new StartEndCommand(accelerator::spinup, accelerator::stop)
        );
        addRequirements(feeder, shooter, accelerator);
    }
}
