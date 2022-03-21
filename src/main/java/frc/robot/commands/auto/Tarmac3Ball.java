package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFenderShotCommand;
import frc.robot.commands.PreAimCommand;
import frc.robot.commands.climber.DeployIntakeCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/** Basic 1 ball auto, fender shot, then taxi */
public class Tarmac3Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 4.0; // m/s
  private static final double MAX_ACCEL = 5.0; // m/s^2

  private static final double SECOND_SHOT_ANGLE = Math.toRadians(-30.768);
  private static final double SECOND_SHOT_RANGE = 0.8; // meters

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory =
      PathPlanner.loadPath("Tarmac3Ball", MAX_VEL, MAX_ACCEL);

  public Tarmac3Ball(
      Swerve swerve,
      Shooter shooter,
      Turret turret,
      Hood hood,
      Accelerator accelerator,
      Feeder feeder,
      Intake intake,
      Climber climber) {
    addCommands(
        new AutoCommands.ResetPose(trajectory, swerve),
        new ParallelCommandGroup(
          new AutoFenderShotCommand(shooter, hood, turret, accelerator, feeder).withTimeout(1.5),
          new DeployIntakeCommand(climber)
        ),
        new ParallelCommandGroup(
            new PreAimCommand(hood, turret, shooter, SECOND_SHOT_ANGLE, SECOND_SHOT_RANGE),
            new AutoCommands.IntakeWhileDriving(trajectory, swerve, intake, feeder, accelerator)
        ),
       new AutoCommands.ShootWithPreAim(feeder, accelerator)
    );
  }
}
