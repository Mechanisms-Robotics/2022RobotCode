package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LowGoalCommand;
import frc.robot.commands.PreAimCommand;
import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.SetIntakeCommand.IntakeMode;
import frc.robot.commands.accelerator.AcceleratorShootCommand;
import frc.robot.commands.feeder.FeederShootCommand;
import frc.robot.commands.shooter.ShooterAimCommand;
import frc.robot.commands.shooter.ShooterEjectCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
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
      Accelerator accelerator,
      Feeder feeder,
      Intake intake,
      Limelight limelight) {
    addCommands(
        new AutoCommands.ResetPose(trajectory1, swerve),
        new SetIntakeCommand(intake, IntakeMode.DEPLOY),
        new ParallelRaceGroup(
          new ShooterAimCommand(shooter, () -> limelight.getCurrentTarget().hasTarget, () -> limelight.getCurrentTarget().targetAngle),
          new AutoCommands.IntakeWhileDriving(trajectory1, swerve, intake, feeder, accelerator)
        ),
        new ParallelRaceGroup(
            new ShooterAimCommand(shooter, () -> limelight.getCurrentTarget().hasTarget, () -> limelight.getCurrentTarget().targetAngle),
            new AutoCommands.ShootWithPreAim(feeder, accelerator)
        ),
        new ParallelRaceGroup(
            new ShooterAimCommand(shooter, () -> limelight.getCurrentTarget().hasTarget, () -> limelight.getCurrentTarget().targetAngle),
            new AutoCommands.IntakeWhileDriving(trajectory2, swerve, intake, feeder, accelerator)
        ),
        new ParallelCommandGroup(
            new ShooterEjectCommand(shooter),
            new AcceleratorShootCommand(accelerator, shooter::getRPM),
            new FeederShootCommand(feeder, shooter::atSpeed)
        ).withTimeout(3.0),
        new AutoCommands.FollowPathCommand(trajectory3, swerve));
  }
}
