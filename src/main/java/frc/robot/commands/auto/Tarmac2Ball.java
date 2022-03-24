package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PreAimCommand;
import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.SetIntakeCommand.IntakeMode;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/** Basic 1 ball auto, fender shot, then taxi */
public class Tarmac2Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 2.0; // m/s
  private static final double MAX_ACCEL = 4.0; // m/s^2

  private static final double FIRST_SHOT_ANGLE = -1.0; // rads
  private static final double FIRST_SHOT_RANGE = 0.25;

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory =
      PathPlanner.loadPath("Tarmac2Ball", MAX_VEL, MAX_ACCEL);

  public Tarmac2Ball(
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
        new SetIntakeCommand(intake, IntakeMode.DEPLOY),
        new ParallelCommandGroup(
            new AutoCommands.IntakeWhileDriving(trajectory, swerve, intake, feeder, accelerator),
            new PreAimCommand(hood, turret, shooter, FIRST_SHOT_ANGLE, FIRST_SHOT_RANGE)),
        new AutoCommands.ShootWithPreAim(feeder, accelerator));
  }
}
