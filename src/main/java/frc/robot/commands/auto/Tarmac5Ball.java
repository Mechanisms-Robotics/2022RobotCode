package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PreAimCommand;
import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.SetIntakeCommand.IntakeMode;
import frc.robot.commands.auto.AutoCommands.ShootWithPreAim;
import frc.robot.commands.hood.HoodAimCommand;
import frc.robot.commands.shooter.ShooterAimCommand;
import frc.robot.commands.turret.TurretAimCommand;
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
public class Tarmac5Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 4.5; // m/s
  private static final double MAX_ACCEL = 3.5; // m/s^2

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory1 =
      PathPlanner.loadPath("Tarmac3Ball", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory trajectory2 =
      PathPlanner.loadPath("Tarmac5Ball", MAX_VEL, MAX_ACCEL);

  public Tarmac5Ball(
      Swerve swerve,
      Accelerator accelerator,
      Feeder feeder,
      Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new AutoCommands.ResetPose(trajectory1, swerve),
            new SetIntakeCommand(intake, IntakeMode.DEPLOY)
        ),
        new ShootWithPreAim(feeder, accelerator, 1.0),
        new AutoCommands.IntakeWhileDriving(trajectory1, swerve, intake, feeder, accelerator),
        new AutoCommands.ShootWithPreAim(feeder, accelerator, 2.0),
        new AutoCommands.IntakeWhileDriving(trajectory2, swerve, intake, feeder, accelerator),
        new AutoCommands.ShootWithPreAim(feeder, accelerator, 3.0));
  }
}
