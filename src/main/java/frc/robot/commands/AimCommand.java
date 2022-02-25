package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.GoalTracker;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/** This command adjusts the shooter RPM, hood angle, and turret angle relative to a target. */
public class AimCommand extends CommandBase {
  private final Shooter shooter;
  private final Hood hood;
  private final Turret turret;
  private final GoalTracker goalTracker;

  /**
   * Constructs an AimCommand
   *
   * @param shooter Instance of Shooter
   * @param hood Instance of Hood
   * @param turret Instance of Turret
   * @param goalTracker Instance of GoalTracker
   */
  public AimCommand(Shooter shooter, Hood hood, Turret turret, GoalTracker goalTracker) {
    this.shooter = shooter;
    this.hood = hood;
    this.turret = turret;
    this.goalTracker = goalTracker;

    // Add the shooter, hood, turret, and goalTracker as requirements
    addRequirements(shooter, hood, turret, goalTracker);
  }

  @Override
  public void execute() {
    if (goalTracker.hasTarget()) {
      turret.aim(goalTracker.getTargetAngle());
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }
}
