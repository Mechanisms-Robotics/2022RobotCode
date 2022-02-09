package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;

public class FeederCommand extends CommandBase {
  private final Feeder feeder;

  public FeederCommand(Feeder feeder) {
    this.feeder = feeder;

    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    feeder.setOpenLoop(Constants.feederSpeed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }
}
