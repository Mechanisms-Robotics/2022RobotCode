package frc.robot.commands.accelerator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accelerator;

public class AcceleratorCommand extends CommandBase {
  private final Accelerator accelerator;

  public AcceleratorCommand(Accelerator accelerator) {
    this.accelerator = accelerator;

    addRequirements(accelerator);
  }

  @Override
  public void initialize() {
    accelerator.setOpenLoop(Constants.acceleratorSpeed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    accelerator.stop();
  }
}
