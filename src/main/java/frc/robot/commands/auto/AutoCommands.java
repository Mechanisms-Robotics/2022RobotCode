package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.feeder.FeederIntakeCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

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
          new ParallelCommandGroup(new IntakeCommand(intake), new FeederIntakeCommand(feeder)));
    }
  }

  public static class PreAimCommand extends CommandBase {

    protected final double preAimAngle;
    protected final double preAimRange;
    protected final Hood hood;
    protected final Turret turret;
    protected final Shooter shooter;

    public PreAimCommand(
        Hood hood, Turret turret, Shooter shooter, double preAimAngle, double preAimRange) {
      this.preAimAngle = preAimAngle;
      this.preAimRange = preAimRange;
      this.hood = hood;
      this.turret = turret;
      this.shooter = shooter;
      addRequirements(turret, hood, shooter);
    }

    @Override
    public void initialize() {
      hood.aim(preAimRange);
      turret.snapTo(preAimAngle);
      shooter.shoot(preAimRange);
    }

    @Override
    public boolean isFinished() {
      return turret.isAimed() && shooter.atSpeed();
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
