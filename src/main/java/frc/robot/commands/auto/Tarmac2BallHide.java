package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoCommands.PreAimCommand;
import frc.robot.commands.intake.IntakeDeployCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/** Basic 1 ball auto, fender shot, then taxi */
public class Tarmac2BallHide extends SequentialCommandGroup {

  private static final double MAX_VEL = 2.0; // m/s
  private static final double MAX_ACCEL = 4.0; // m/s^2

  private static final double FIRST_SHOT_ANGLE = Math.toRadians(-75.0);
  private static final double FIRST_SHOT_RANGE = 0.6;

  private static final double SECOND_SHOT_ANGLE = Math.toRadians(0.0);
  private static final double SECOND_SHOT_RANGE = 20.0; // Low goal, hood all the way forward

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory1 =
      PathPlanner.loadPath("Tarmac2Ball", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory trajectory2 =
      PathPlanner.loadPath("Tarmac2BallHide", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory trajectory3 =
      PathPlanner.loadPath("TarmacPrepPickup", MAX_VEL, MAX_ACCEL);

  public Tarmac2BallHide(
      Swerve swerve,
      Shooter shooter,
      Turret turret,
      Hood hood,
      Accelerator accelerator,
      Feeder feeder,
      Intake intake,
      Climber climber) {
    addCommands(
        new AutoCommands.ResetPose(trajectory1, swerve),
        new IntakeDeployCommand(intake),
        new ParallelCommandGroup(
            new AutoCommands.IntakeWhileDriving(trajectory1, swerve, intake, feeder, accelerator),
            new PreAimCommand(hood, turret, shooter, FIRST_SHOT_ANGLE, FIRST_SHOT_RANGE)),
        new AutoCommands.ShootWithPreAim(feeder, accelerator, 3.0),
        new ParallelCommandGroup(
            new AutoCommands.IntakeWhileDriving(trajectory2, swerve, intake, feeder, accelerator),
            new PreAimCommand(hood, turret, shooter, SECOND_SHOT_ANGLE, SECOND_SHOT_RANGE)),
        //        new LowGoalCommand(shooter, hood, turret, accelerator, feeder).withTimeout(3.0),
        new AutoCommands.FollowPathCommand(trajectory3, swerve));
  }
}
