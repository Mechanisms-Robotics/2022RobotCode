package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FenderShotCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/**
 * Basic 1 ball auto, fender shot, then taxi
 */
public class Basic1Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 1.0; // m/s
  private static final double MAX_ACCEL = 2.0; // m/s^2

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory =
      PathPlanner.loadPath("Basic1Ball", MAX_VEL, MAX_ACCEL);

  public Basic1Ball(Swerve swerve, Shooter shooter, Turret turret, Accelerator accelerator, Feeder feeder) {
    addCommands(
        new FenderShotCommand(shooter, turret, accelerator, feeder).withTimeout(3.0),
        new InstantCommand(
            () ->
                swerve.setPose(
                    trajectory.getInitialPose(), trajectory.getInitialState().holonomicRotation))
                    ,
                new FunctionalCommand(
                    () -> swerve.followTrajectory(trajectory),
                    () -> {},
                    interrupted -> {},
                    swerve::isTrajectoryFinished,
                    swerve).andThen(swerve::stop)
        );
  }
}
