package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BackupCommand;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/** Basic 1 ball auto, fender shot, then taxi */
public class TarmacFender2Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 1.0; // m/s
  private static final double MAX_ACCEL = 2.0; // m/s^2

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory toBall =
      PathPlanner.loadPath("Tarmac2Ball", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory toFender =
      PathPlanner.loadPath("Tarmac2BallFender", MAX_VEL, MAX_ACCEL);

  public TarmacFender2Ball(
      Swerve swerve,
      Shooter shooter,
      Turret turret,
      Hood hood,
      Accelerator accelerator,
      Feeder feeder,
      Intake intake) {
    addCommands(
        new InstantCommand(
            () ->
                swerve.setPose(
                    toBall.getInitialPose(), toBall.getInitialState().holonomicRotation)),
        new IntakeCommand(intake, feeder, accelerator)
            .raceWith(
                new SequentialCommandGroup(
                    new FunctionalCommand(
                            () -> swerve.followTrajectory(toBall),
                            () -> {},
                            interrupted -> {},
                            swerve::isTrajectoryFinished,
                            swerve)
                        .andThen(swerve::stop),
                    new FunctionalCommand(
                            () -> swerve.followTrajectory(toFender),
                            () -> {},
                            interrupted -> {},
                            swerve::isTrajectoryFinished,
                            swerve)
                        .andThen(swerve::stop)))
            .andThen(new BackupCommand(accelerator, feeder)),
        new ParallelCommandGroup(
            new AimCommand(turret, hood, () -> false, () -> 0.0, () -> 0.0, () -> true),
            new FenderShotCommand(shooter, turret, accelerator, feeder)));
  }
}
