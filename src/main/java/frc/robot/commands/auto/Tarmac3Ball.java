package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PreAimCommand;
import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.SetIntakeCommand.IntakeMode;
import frc.robot.commands.auto.AutoCommands.ShootWithPreAim;
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

  private static final double MAX_VEL = 2.0; // m/s
  private static final double MAX_ACCEL = 4.0; // m/s^2

  private static final double FIRST_SHOT_ANGLE = -1.83; // rads
  private static final double FIRST_SHOT_RANGE = 0.25; // meters

  private static final double SECOND_SHOT_ANGLE = -0.14;
  private static final double SECOND_SHOT_RANGE = 0.19; // meters

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
        new ParallelCommandGroup(
            new AutoCommands.ResetPose(trajectory, swerve),
            new SetIntakeCommand(intake, IntakeMode.DEPLOY),
            new PreAimCommand(hood, turret, shooter, FIRST_SHOT_ANGLE, FIRST_SHOT_RANGE)),
        new WaitCommand(0.625),
        new ShootWithPreAim(feeder, accelerator, 2.0),
        new ParallelCommandGroup(
            new PreAimCommand(hood, turret, shooter, SECOND_SHOT_ANGLE, SECOND_SHOT_RANGE),
            new AutoCommands.IntakeWhileDriving(trajectory, swerve, intake, feeder, accelerator)),
        new AutoCommands.ShootWithPreAim(feeder, accelerator, 6.0));
  }
}
