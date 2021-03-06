package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.SetIntakeCommand.IntakeMode;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

/** Basic 1 ball auto, fender shot, then taxi */
public class Tarmac2Ball extends SequentialCommandGroup {

  private static final double MAX_VEL = 4.0; // m/s
  private static final double MAX_ACCEL = 2.0; // m/s^2

  // TODO: find maxVel and maxAccel
  private static final PathPlannerTrajectory trajectory =
      PathPlanner.loadPath("Tarmac2Ball", MAX_VEL, MAX_ACCEL);

  public Tarmac2Ball(Swerve swerve, Accelerator accelerator, Feeder feeder, Intake intake) {
    addCommands(
        new AutoCommands.ResetPose(trajectory, swerve),
        new SetIntakeCommand(intake, IntakeMode.DEPLOY),
        new AutoCommands.IntakeWhileDriving(trajectory, swerve, intake, feeder, accelerator),
        new AutoCommands.ShootWithPreAim(feeder, accelerator));
  }
}
