package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import org.photonvision.PhotonCamera;

public class Basic1Ball extends SequentialCommandGroup {

    private static final double MAX_VEL = 1.0;
    private static final double MAX_ACCEL = 2.0;

    // TODO: find maxVel and maxAccel
    private static final PathPlannerTrajectory trajectory =
            PathPlanner.loadPath("Basic1Ball", MAX_VEL, MAX_ACCEL);

    public Basic1Ball(Swerve swerve) {
        addCommands(new RunCommand(() -> swerve.followTrajectory(trajectory), swerve)); // TODO: use FunctionalCommand
    }

}
