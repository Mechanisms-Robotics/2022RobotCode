package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class Basic1Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 1.0;
  private static final double MAX_ACCEL = 2.0;

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory =
      PathPlanner.loadPath("Basic1Ball", MAX_VEL, MAX_ACCEL);

  public Basic1Ball(Swerve swerve) {
    addCommands(
        new InstantCommand(
            () ->
                swerve.setPose(
                    trajectory.getInitialPose(), trajectory.getInitialState().holonomicRotation))
        //            ,
        //        new FunctionalCommand(
        //            () -> swerve.followTrajectory(trajectory),
        //            () -> {},
        //            interrupted -> {},
        //            () -> false,
        //            swerve)
        );
  }
}
