package frc.robot.commands;

import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/** This command runs the shooter, accelerator, and feeder to shoot a ball */
public class FenderShotCommand extends AutoFenderShotCommand {

  /**
   * Constructs a ShootCommand
   *
   * @param shooter     Instance of Shooter
   * @param turret      Instants of the Turret
   * @param accelerator Instance of Accelerator
   * @param feeder      Instance of Feeder
   */
  public FenderShotCommand(Shooter shooter, Hood hood, Turret turret, Accelerator accelerator,
                           Feeder feeder) {
    super(shooter, hood, turret, accelerator, feeder);
  }

  @Override
  public void end(boolean interrupted) {
    // Stops the feeder, accelerator, shooter, and turret
    super.end(interrupted);
    super.shooter.stop();
  }
}
