package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;

public class PrepFeederCommand extends CommandBase {

  private final Feeder feeder;
  private final Accelerator accelerator;

  public PrepFeederCommand(Feeder feeder, Accelerator accelerator) {
    this.feeder = feeder;
    this.accelerator = accelerator;
    addRequirements(this.feeder, this.accelerator);
  }

  @Override
  public void initialize() {
    feeder.intake();
    accelerator.prep();
  }

  @Override
  public boolean isFinished() {
    return accelerator.isPreped();
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    accelerator.stop();
  }
}
