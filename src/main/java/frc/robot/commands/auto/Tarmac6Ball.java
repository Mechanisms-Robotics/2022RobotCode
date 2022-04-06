package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.SetIntakeCommand.IntakeMode;
import frc.robot.commands.auto.AutoCommands.ShootWithPreAim;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

/** Basic 1 ball auto, fender shot, then taxi */
public class Tarmac6Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 4.5; // m/s
  private static final double MAX_ACCEL = 3.5; // m/s^2

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory1 =
      PathPlanner.loadPath("Tarmac3Ball", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory trajectory2 =
      PathPlanner.loadPath("Tarmac6Ball", MAX_VEL, MAX_ACCEL);

  public Tarmac6Ball(Swerve swerve, Accelerator accelerator, Feeder feeder, Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new AutoCommands.ResetPose(trajectory1, swerve),
            new SetIntakeCommand(intake, IntakeMode.DEPLOY)),
        new ShootWithPreAim(feeder, accelerator, 1.0),
        new AutoCommands.IntakeWhileDriving(trajectory1, swerve, intake, feeder, accelerator),
        new AutoCommands.ShootWithPreAim(feeder, accelerator, 2.0),
        new AutoCommands.IntakeWhileDriving(trajectory2, swerve, intake, feeder, accelerator),
        new AutoCommands.ShootWithPreAim(feeder, accelerator, 3.0));
  }
}
