package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFenderShotCommand;
import frc.robot.commands.PreAimCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/** Basic 1 ball auto, fender shot, then taxi */
public class Tarmac5Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 1.0; // m/s
  private static final double MAX_ACCEL = 2.0; // m/s^2

  private static final double SECOND_SHOT_ANGLE = Math.toRadians(-25);
  private static final double SECOND_SHOT_RANGE = 0.6; // meters

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
      Hood hood,
      Feeder feeder,
      Intake intake) {
    addCommands(
        new AutoCommands.ResetPose(trajectory1, swerve),
        new AutoFenderShotCommand(shooter, hood, turret, accelerator, feeder).withTimeout(1.5),
        new ParallelCommandGroup(
            new PreAimCommand(hood, turret, shooter, SECOND_SHOT_ANGLE, SECOND_SHOT_RANGE),
            new AutoCommands.IntakeWhileDriving(trajectory1, swerve, intake, feeder, accelerator)
        ),
        new AutoCommands.ShootWithPreAim(feeder, accelerator),
        new InstantCommand(shooter::stop, shooter)
        // TODO: Add 5 ball path and shooting
    );
  }
}
