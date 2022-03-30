package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/** This command runs the shooter, accelerator, and feeder to shoot a ball */
public class ShootCommand extends CommandBase {
  private final Shooter shooter;
  private final Accelerator accelerator;
  private final Feeder feeder;
  private final Limelight limelight;

  // The amount of time it takes to spinup
  private static final double SPINUP_TIME = 1.0; // seconds

  /**
   * Constructs a ShootCommand
   *
   * @param shooter Instance of Shooter
   * @param accelerator Instance of Accelerator
   * @param feeder Instance of Feeder
   */
  public ShootCommand(
      Shooter shooter, Accelerator accelerator, Feeder feeder, Limelight limelight) {
    this.shooter = shooter;
    this.accelerator = accelerator;
    this.feeder = feeder;
    this.limelight = limelight;

    // Add the shooter, accelerator, and feeder as a requirement
    addRequirements(shooter, accelerator, feeder);
  }

  @Override
  public void initialize() {
    var target = limelight.getCurrentTarget();
    if (target.hasTarget) {
      shooter.shoot(target.range);
    } else {
      shooter.shoot();
    }

    accelerator.shoot();
  }

  @Override
  public void execute() {
    if (shooter.atSpeed()) {
      feeder.shoot();
    } else {
      feeder.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    accelerator.stop();
    shooter.stop();
  }
}
