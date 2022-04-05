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
import frc.robot.commands.turret.TurretAimCommand;
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
  private static final double MAX_ACCEL = 2.0; // m/s^2

  private static final double FIRST_SHOT_ANGLE = -2.0; // rads
  private static final double FIRST_SHOT_RANGE = 0.1885; // meters

  private static final double SECOND_SHOT_ANGLE = -0.47; // rads
  private static final double SECOND_SHOT_RANGE = 0.7475; // meters

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory =
      PathPlanner.loadPath("Tarmac3Ball", MAX_VEL, MAX_ACCEL);

  public Tarmac3Ball(
      Swerve swerve,
      Accelerator accelerator,
      Feeder feeder,
      Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new AutoCommands.ResetPose(trajectory, swerve),
            new SetIntakeCommand(intake, IntakeMode.DEPLOY)
        ),
        new ShootWithPreAim(feeder, accelerator, 1.0),
        new AutoCommands.IntakeWhileDriving(trajectory, swerve, intake, feeder, accelerator),
        new AutoCommands.ShootWithPreAim(feeder, accelerator, 3.0));
  }
}
