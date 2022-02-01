package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.accelerator.AcceleratorCommand;
import frc.robot.commands.feeder.FeederCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends ParallelCommandGroup {
  public final Feeder feeder;
  public final Accelerator accelerator;
  public final Shooter shooter;

  public ShootCommand(Feeder feeder, Accelerator accelerator, Shooter shooter) {
    this.feeder = feeder;
    this.accelerator = accelerator;
    this.shooter = shooter;

    addCommands(
        new SpinupCommand(shooter), new AcceleratorCommand(accelerator), new FeederCommand(feeder));
  }
}
