package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BackupCommand;
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
public class Tarmac2Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 1.0; // m/s
  private static final double MAX_ACCEL = 2.0; // m/s^2

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory =
      PathPlanner.loadPath("Tarmac2Ball", MAX_VEL, MAX_ACCEL);

  public Tarmac2Ball(
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
                    trajectory.getInitialPose(), trajectory.getInitialState().holonomicRotation)),
        new IntakeCommand(intake, feeder, accelerator)
            .raceWith(
                new FunctionalCommand(
                        () -> swerve.followTrajectory(trajectory),
                        () -> {},
                        interrupted -> {},
                        swerve::isTrajectoryFinished,
                        swerve)
                    .andThen(swerve::stop))
            .andThen(new BackupCommand(accelerator, feeder)),
        new RunCommand(() -> turret.snapTo(Math.toRadians(-90.0))).withTimeout(1.0),
        new ParallelCommandGroup(
            new AimCommand(
                turret,
                hood,
                hasTargetSupplier,
                targetAngleSupplier,
                targetRangeSupplier,
                () -> false),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new ShootCommand(
                    shooter, accelerator, feeder, hasTargetSupplier, targetRangeSupplier))));
  }
}
