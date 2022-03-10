package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class LowGoalCommand extends CommandBase {
  // Subsystems
  protected final Shooter shooter;
  protected final Turret turret;
  protected final Accelerator accelerator;
  protected final Feeder feeder;
  protected final Hood hood;

  /**
   * Constructs a LowGoalCommand
   *
   * @param shooter Instance of Shooter
   * @param accelerator Instance of Accelerator
   * @param feeder Instance of Feeder
   */
  public LowGoalCommand(Shooter shooter, Hood hood, Turret turret, Accelerator accelerator, Feeder feeder) {
    this.shooter = shooter;
    this.turret = turret;
    this.accelerator = accelerator;
    this.feeder = feeder;
    this.hood = hood;

    // Add the shooter, accelerator, and feeder as a requirement
    addRequirements(shooter, accelerator, feeder, hood, turret);
  }

  /** Initializes the LowGoalCommand */
  @Override
  public void initialize() {
    // Set the turret to the zero position
    turret.goToZero();
    hood.aim(20.0);

    // Run the shooter and the accelerator in shoot mode
    shooter.shootLowGoal();
    accelerator.shoot();
  }

  /** Periodically called while FenderShotCommand is running */
  @Override
  public void execute() {
    // Waits for the timer to have elapsed SPINUP_TIME
    if (shooter.atSpeed()) {
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
    // Stops the feeder, accelerator, and turret
    feeder.stop();
    accelerator.stop();
    shooter.stop();
  }
}