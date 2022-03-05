package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/** This command runs the shooter, accelerator, and feeder to shoot a ball */
public class FenderShotCommand extends CommandBase {
  // Subsystems
  private final Shooter shooter;
  private final Turret turret;
  private final Accelerator accelerator;
  private final Feeder feeder;

  // Timer to wait for spinup
  private final Timer spinupTimer = new Timer();

  // The amount of time it takes to spinup
  private static final double SPINUP_TIME = 1.0; // seconds

  /**
   * Constructs a ShootCommand
   *
   * @param shooter Instance of Shooter
   * @param accelerator Instance of Accelerator
   * @param feeder Instance of Feeder
   */
  public FenderShotCommand(Shooter shooter, Turret turret, Accelerator accelerator, Feeder feeder) {
    this.shooter = shooter;
    this.turret = turret;
    this.accelerator = accelerator;
    this.feeder = feeder;

    // Add the shooter, accelerator, and feeder as a requirement
    addRequirements(shooter, accelerator, feeder);
  }

  /** Initializes the FenderShotCommand */
  @Override
  public void initialize() {
    // Set the turret to the zero position
    turret.goToZero();

    // Run the shooter and the accelerator in shoot mode
    shooter.shoot();
    accelerator.shoot();

    // Start the spinup timer
    spinupTimer.reset();
    spinupTimer.start();
  }

  /** Periodically called while FenderShotCommand is running */
  @Override
  public void execute() {
    // Waits for the timer to have elapsed SPINUP_TIME
    if (spinupTimer.hasElapsed(SPINUP_TIME)) {
      // Runs the feeder in shoot mode
      feeder.shoot();
    }
  }

  /**
   * Called when the FenderShotCommand ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stops the feeder, accelerator, shooter, and turret
    feeder.stop();
    accelerator.stop();
    shooter.stop();
    turret.stop();
  }
}
