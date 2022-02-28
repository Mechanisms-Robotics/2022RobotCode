package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BackupCommand;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** Basic 1 ball auto, fender shot, then taxi */
public class Tarmac5Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 1.0; // m/s
  private static final double MAX_ACCEL = 2.0; // m/s^2

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory1 =
      PathPlanner.loadPath("Tarmac5Ball1", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory trajectory2 =
      PathPlanner.loadPath("Tarmac5Ball2", MAX_VEL, MAX_ACCEL);

  public Tarmac5Ball(
      Swerve swerve,
      Shooter shooter,
      Turret turret,
      Accelerator accelerator,
      Feeder feeder,
      Intake intake,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetRangeSupplier) {
    addCommands(
        new InstantCommand(
            () ->
                swerve.setPose(
                    trajectory1.getInitialPose(), trajectory1.getInitialState().holonomicRotation)),
        new FenderShotCommand(shooter, turret, accelerator, feeder)
            .withTimeout(1.5),
        new IntakeCommand(intake, feeder, accelerator)
            .raceWith(
                new FunctionalCommand(
                    () -> swerve.followTrajectory(trajectory1),
                    () -> {},
                    interrupted -> {},
                    swerve::isTrajectoryFinished,
                    swerve)
                    .andThen(swerve::stop))
            .andThen(new BackupCommand(accelerator, feeder)),
        new ShootCommand(shooter, accelerator, feeder, hasTargetSupplier, targetRangeSupplier).withTimeout(2.0),
        new IntakeCommand(intake, feeder, accelerator).raceWith(
            new FunctionalCommand(
                () -> swerve.followTrajectory(trajectory2),
                () -> {},
                interrupted -> {},
                swerve::isTrajectoryFinished,
                swerve)
                .andThen(swerve::stop)
        ).andThen(new BackupCommand(accelerator, feeder)),
        new ShootCommand(shooter, accelerator, feeder, hasTargetSupplier, targetRangeSupplier));
  }
}
