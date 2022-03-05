package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BackupCommand;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** Basic 1 ball auto, fender shot, then taxi */
public class Tarmac5Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 4.5; // m/s
  private static final double MAX_ACCEL = 5.0; // m/s^2

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory1 =
      PathPlanner.loadPath("Tarmac3Ball", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory trajectory2 =
      PathPlanner.loadPath("Tarmac5Ball", MAX_VEL, MAX_ACCEL);

  public Tarmac5Ball(
      Swerve swerve,
      Shooter shooter,
      Turret turret,
      Hood hood,
      Accelerator accelerator,
      Feeder feeder,
      Intake intake,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Double> targetAngleSupplier,
      Supplier<Double> targetRangeSupplier) {
    addCommands(
        new InstantCommand(
            () ->
                swerve.setPose(
                    trajectory1.getInitialPose(), trajectory1.getInitialState().holonomicRotation)),
        new AimCommand(turret, hood, () -> false, () -> 0.0, () -> 0.0, () -> true)
            .raceWith(
//                new FenderShotCommand(shooter, turret, accelerator, feeder).withTimeout(1.25)
                new RunCommand(shooter::shoot),
                new RunCommand(accelerator::shoot)
            ).withTimeout(0.75).andThen(
                new StartEndCommand(feeder::shoot, feeder::stop).withTimeout(1.25)
            ),
        new AimCommand(turret, hood, hasTargetSupplier, targetAngleSupplier, targetRangeSupplier, () -> false).raceWith(
            new StartEndCommand(intake::intake, intake::stop),
            new StartEndCommand(feeder::intake, feeder::stop),
            new StartEndCommand(accelerator::idle, accelerator::stop),
            new FunctionalCommand(
                () -> swerve.followTrajectory(trajectory1),
                () -> {},
                interrupted -> {},
                swerve::isTrajectoryFinished,
                swerve)
                .andThen(swerve::stop)
        ),
        new WaitCommand(0.5),
        new ParallelCommandGroup(
            new InstantCommand(() -> shooter.shoot(targetRangeSupplier.get())),
            new InstantCommand(accelerator::backup),
            new InstantCommand(feeder::backup)
        ).withTimeout(1.0),
        new ParallelCommandGroup(
            new InstantCommand(accelerator::shoot),
            new InstantCommand(feeder::shoot)
        ),
        new WaitCommand(3.0),
        new ParallelCommandGroup(
            new InstantCommand(accelerator::stop),
            new InstantCommand(feeder::stop)
        ),
        new InstantCommand(
            () ->
                swerve.setPose(
                    trajectory2.getInitialPose(), trajectory2.getInitialState().holonomicRotation))
            .withTimeout(2.0),
        new FunctionalCommand(
                        () -> swerve.followTrajectory(trajectory2),
                        () -> {},
                        interrupted -> {},
                        swerve::isTrajectoryFinished,
                        swerve)
                    .andThen(swerve::stop));
  }
}
