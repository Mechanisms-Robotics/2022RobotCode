package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AimCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.FenderShotCommand;
import frc.robot.commands.LowGoalCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PrepFeederCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.auto.Tarmac2Ball;
import frc.robot.commands.auto.Tarmac2BallHide;
import frc.robot.commands.auto.Tarmac3Ball;
import frc.robot.commands.auto.Tarmac5Ball;
import frc.robot.commands.auto.Tarmac6Ball;
import frc.robot.commands.climber.DeployIntakeCommand;
import frc.robot.commands.drivetrain.DriveTeleopCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
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
  public final Hood hood = new Hood();
  public final Turret turret = new Turret();
  private final Climber climber = new Climber();

  // Goal Tracker
  private final Limelight limelight = new Limelight();

  // Controllers
  private final ControllerWrapper driverController = new ControllerWrapper(0);
  private final ControllerWrapper secondaryController = new ControllerWrapper(1);

  // Buttons
  private final Button intakeButton = new Button(driverController::getLeftTriggerButton);
  private final Button outtakeButton = new Button(driverController::getTriangleButton);
  private final Button fenderShotButton = new Button(driverController::getRightBumperButton);
  private final Button lowGoalButton = new Button(driverController::getXButton);
  private final Button shootButton = new Button(driverController::getRightTriggerButton);
  private final Button autoShootButton = new Button(driverController::getLeftBumperButton);
  private final Button deployIntakeButton = new Button(driverController::getSquareButton);

  private final Button feederIntakeButton = new Button(secondaryController::getRightBumperButton);
  private final Button feederBackupButton = new Button(secondaryController::getRightTriggerButton);
  private final Button backupShooterButton = new Button(secondaryController::getXButton);

  private final Button climberButtonUp = new Button(() -> secondaryController.getPOV() == ControllerWrapper.Direction.Up);
  private final Button climberButtonDown = new Button(() -> secondaryController.getPOV() == ControllerWrapper.Direction.Down);

  private final Button gyroResetButton = new Button(driverController::getShareButton);

  // Swerve inputs
  Supplier<Double> inputX = () -> -driverController.getLeftJoystickX();
  Supplier<Double> inputY = () -> driverController.getLeftJoystickY();
  Supplier<Double> inputRotation = () -> -driverController.getRightJoystickX();

  /** Constructs a RobotContainer */
  public RobotContainer() {
    // Zero the swerve heading
    swerve.zeroHeading();

    // Configure button bindings
    configureButtonBindings();

    // Configure default commands
    configureDefaultCommands();

    autoChooser.setDefaultOption(Autos.TARMAC_3_BALL.name(), Autos.TARMAC_3_BALL);
    autoChooser.addOption(Autos.TARMAC_2_BALL.name(), Autos.TARMAC_2_BALL);
    autoChooser.addOption(Autos.TARMAC_3_BALL.name(), Autos.TARMAC_3_BALL);
    SmartDashboard.putData(autoChooser);
  }

  /** Configures all default commands */
  private void configureDefaultCommands() {
    // Set the swerve default command to a DriveTeleopCommand
    swerve.setDefaultCommand(new DriveTeleopCommand(inputX, inputY, inputRotation, true, swerve));

    // Set the turret default command to a AimCommand
    turret.setDefaultCommand(
        new AimCommand(
            swerve,
            turret,
            hood,
            limelight,
            fenderShotButton::get));
  }

  /** Configures all button bindings */
  private void configureButtonBindings() {
    // When the intake button is pressed toggle the intake.
    intakeButton.toggleWhenPressed(new AutoIntakeCommand(intake, feeder, accelerator));

    // When the D-Pad is held up run the shooter up
    climberButtonUp.whenHeld(new StartEndCommand(climber::up, climber::stop));

    // When the D-Pad is held down run the shooter down
    climberButtonDown.whenHeld(new StartEndCommand(climber::down, climber::stop));

    // When the outtake button is held run an OuttakeCommand
    outtakeButton.whenHeld(new OuttakeCommand(intake, feeder, accelerator));

    // When the fender shot button is held, run a FenderShotCommand
    fenderShotButton.whenHeld(new FenderShotCommand(shooter, hood, turret, accelerator, feeder));

    // When the low goal button is held, run a LowGoalCommand
    lowGoalButton.whenHeld(new LowGoalCommand(shooter, hood, turret, accelerator, feeder));

    // When the shoot button is pressed toggle a ShootCommand
    shootButton.whenHeld(
        new ShootCommand(
            shooter, accelerator, feeder, limelight));

    // When the auto shoot button is pressed toggle a AutoShoot command
    autoShootButton.toggleWhenPressed(
        new AutoShootCommand(
            shooter,
            accelerator,
            feeder,
            turret,
            limelight,
            swerve));

    // When the feeder intake button is pressed intake the feeder, when it is released stop it
    feederIntakeButton.whenHeld(new StartEndCommand(feeder::intake, feeder::stop));

    // When the feeder backup button is pressed backup the feeder, when it is released stop it
    feederBackupButton.whenHeld(new StartEndCommand(feeder::backup, feeder::stop));

    // When the deploy intake button is pressed, deploy the intake
    deployIntakeButton.whenPressed(new DeployIntakeCommand(climber));

    backupShooterButton.whenHeld(new StartEndCommand(shooter::backup, shooter::stop));

    // When the gyro reset button is pressed run an InstantCommand that zeroes the swerve heading
    gyroResetButton.whenPressed(new InstantCommand(swerve::zeroHeading));
  }

  public enum Autos {
    TARMAC_2_BALL,
    TARMAC_2_BALL_HIDE,
    TARMAC_3_BALL,
    TARMAC_5_BALL,
    TARMAC_6_BALL
  }

  private SendableChooser<Autos> autoChooser = new SendableChooser<>();

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case TARMAC_2_BALL:
        return new Tarmac2Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      case TARMAC_2_BALL_HIDE:
        return new Tarmac2BallHide(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      case TARMAC_3_BALL:
        return new Tarmac3Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      case TARMAC_5_BALL:
        return new Tarmac5Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      case TARMAC_6_BALL:
        return new Tarmac6Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      default:
        return new Tarmac3Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
    }
  }
}
