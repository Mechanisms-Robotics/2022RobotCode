package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public final class AutoCommands {

  public static class ResetPose extends InstantCommand {
    public ResetPose(PathPlannerTrajectory trajectory, Swerve swerve) {
      super(
          () ->
              swerve.setPose(
                  trajectory.getInitialPose(), trajectory.getInitialState().holonomicRotation));
    }
  }

  public static class FollowPathCommand extends SequentialCommandGroup {
    public FollowPathCommand(PathPlannerTrajectory trajectory, Swerve swerve) {
      addCommands(
          new FunctionalCommand(
              () -> swerve.followTrajectory(trajectory),
              () -> {},
              interrupted -> {},
              swerve::isTrajectoryFinished,
              swerve),
          new InstantCommand(swerve::stop, swerve));
    }
  }

  public static class IntakeWhileDriving extends ParallelDeadlineGroup {
    public IntakeWhileDriving(
        PathPlannerTrajectory trajectory,
        Swerve swerve,
        Intake intake,
        Feeder feeder,
        Accelerator accelerator) {
      super(
          new FollowPathCommand(trajectory, swerve),
          new AutoIntakeCommand(intake, feeder, accelerator));
    }
  }

  public static class ShootWithPreAim extends SequentialCommandGroup {
    private static final double DEFAULT_SHOOT_TIME = 2.0;

    public ShootWithPreAim(Feeder feeder, Accelerator accelerator, double shootTime) {
      addCommands(
          new ParallelDeadlineGroup(
              new WaitCommand(shootTime),
              new InstantCommand(
                  () -> {
                    accelerator.shoot();
                    feeder.shoot();
                  })),
          new InstantCommand(
              () -> {
                accelerator.stop();
                feeder.stop();
              }));
    }

    public ShootWithPreAim(Feeder feeder, Accelerator accelerator) {
      addCommands(
          new ParallelDeadlineGroup(
              new WaitCommand(DEFAULT_SHOOT_TIME),
              new InstantCommand(
                  () -> {
                    accelerator.shoot();
                    feeder.shoot();
                  })),
          new InstantCommand(
              () -> {
                accelerator.stop();
                feeder.stop();
              }));
    }
  }
}
