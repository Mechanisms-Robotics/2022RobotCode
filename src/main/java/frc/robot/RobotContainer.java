package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BackupCommand;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.auto.Basic1Ball;
import frc.robot.commands.drivetrain.DriveTeleopCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GoalTracker;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.ControllerWrapper;
import java.util.function.Supplier;

public class RobotContainer {

  // Subsystems
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();
  private final Feeder feeder = new Feeder();
  private final Accelerator accelerator = new Accelerator();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  public final Turret turret = new Turret();

  // Goal Tracker
  private final GoalTracker goalTracker = new GoalTracker();

  // Controllers
  private final ControllerWrapper driverController = new ControllerWrapper(0);

  // Buttons
  private final Button intakeButton = new Button(driverController::getLeftTriggerButton);
  private final Button outtakeButton = new Button(driverController::getTriangleButton);
  private final Button fenderShotButton = new Button(driverController::getRightBumperButton);
  private final Button shootButton = new Button(driverController::getRightTriggerButton);

  private final Button gyroResetButton = new Button(driverController::getShareButton);

  // Swerve inputs
  Supplier<Double> inputX = () -> -driverController.getLeftJoystickX(),
      inputY = () -> -driverController.getLeftJoystickY(),
      rotation = () -> -driverController.getRightJoystickX();

  /** Constructs a RobotContainer */
  public RobotContainer() {
    // Zero the swerve heading
    swerve.zeroHeading();

    // Configure button bindings
    configureButtonBindings();

    // Configure default commands
    configureDefaultCommands();
  }

  /** Configures all default commands */
  private void configureDefaultCommands() {
    // Set the swerve default command to a DriveTeleopCommand
    swerve.setDefaultCommand(new DriveTeleopCommand(inputX, inputY, rotation, true, swerve));

    // Set the turret default command to a AimCommand
    turret.setDefaultCommand(
        new AimCommand(
            turret,
            hood,
            goalTracker::hasTarget,
            goalTracker::getTargetAngle,
            goalTracker::getTargetRange,
            fenderShotButton::get));
  }

  /** Configures all button bindings */
  private void configureButtonBindings() {
    // When the intake button is pressed toggle an IntakeCommand, when it is ended run a
    // BackupCommand
    intakeButton.toggleWhenPressed(
        new StartEndCommand(
            () -> {
              new IntakeCommand(intake, feeder, accelerator).schedule();
            },
            () -> {
              new BackupCommand(accelerator, feeder).schedule();
            }));

    // When the outtake button is held run an OuttakeCommand
    outtakeButton.whenHeld(new OuttakeCommand(intake, feeder, accelerator));

    // When the fender shot button is held, run a FenderShotCommand
    fenderShotButton.whenHeld(new FenderShotCommand(shooter, turret, accelerator, feeder));

    // When the shoot button is pressed toggle a ShootCommand
    shootButton.whenHeld(
        new ShootCommand(
            shooter, accelerator, feeder, goalTracker::hasTarget, goalTracker::getTargetRange));

    // When the gyro reset button is pressed run an InstantCommand that zeroes the swerve heading
    gyroResetButton.whenPressed(new InstantCommand(swerve::zeroHeading));
  }

  /**
   * Gets the command to run in autonomous
   *
   * @return The command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Basic1Ball(swerve, shooter, turret, accelerator, feeder);
  }
}
