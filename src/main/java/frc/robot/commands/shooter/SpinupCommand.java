package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SpinupCommand extends CommandBase {
  private final Shooter shooter;

  public SpinupCommand(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setOpenLoop(Constants.flywheelSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
