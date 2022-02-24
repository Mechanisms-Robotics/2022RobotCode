package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.GoalTracker;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/** This command adjusts the shooter RPM, hood angle, and turret angle relative to a target. */
public class AimCommand extends ParallelCommandGroup {

  /**
   * Constructs an AimCommand
   *
   * @param shooter Instance of Shooter
   * @param hood Instance of Hood
   * @param turret Instance of Turret
   * @param goalTracker Instance of GoalTracker
   */
  public AimCommand(Shooter shooter, Hood hood, Turret turret, GoalTracker goalTracker) {
    // Add commands to the ParallelCommandGroup
    addCommands(
        // StartEndCommand to vary the shooter RPM based on range, then stop the shooter
        new StartEndCommand(
            () -> {
              if (goalTracker.hasTarget()) {
                shooter.shoot(goalTracker.getTargetRange());
              }
            },
            shooter::stop),

        // RunCommand to update the hood position based on range
        new RunCommand(
            () -> {
              if (goalTracker.hasTarget()) {
                hood.aimHood(goalTracker.getTargetRange());
              }
            }),

        // StartEndCommand to aim the turret towards the target, then turn off the limelight LEDs
        new StartEndCommand(
            () -> {
              if (goalTracker.hasTarget()) {
                turret.aim(goalTracker.getTargetAngle());
              }
            },
            () -> {
              turret.stop();
              goalTracker.turnOffLEDs();
            }));

    // Add the shooter, hood, turret, and goalTracker as requirements
    addRequirements(shooter, hood, turret, goalTracker);
  }
}
