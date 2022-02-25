package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

/** This command runs the shooter, accelerator, and feeder to shoot a ball */
public class ShootCommand extends CommandBase {
  private final Shooter shooter;
  private final Hood hood;
  private final Accelerator accelerator;
  private final Feeder feeder;

  private final Timer spinupTimer = new Timer();

  // The amount of time it takes to spinup
  private static final double SPINUP_TIME = 0.75; // seconds

  /**
   * Constructs a ShootCommand
   *
   * @param shooter Instance of Shooter
   * @param accelerator Instance of Accelerator
   * @param feeder Instance of Feeder
   */
  public ShootCommand(Shooter shooter, Hood hood, Accelerator accelerator, Feeder feeder) {
    this.shooter = shooter;
    this.hood = hood;
    this.accelerator = accelerator;
    this.feeder = feeder;

    // Add the shooter, accelerator, and feeder as a requirement
    addRequirements(shooter, hood, accelerator, feeder);
  }

  @Override
  public void initialize() {
    hood.setHoodRawPosition(-0.3);

    shooter.shoot();
    accelerator.shoot();

    spinupTimer.reset();
    spinupTimer.start();
  }

  @Override
  public void execute() {
    if (spinupTimer.hasElapsed(SPINUP_TIME)) {
      feeder.shoot();
    }
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    accelerator.stop();
    shooter.stop();
  }
}
