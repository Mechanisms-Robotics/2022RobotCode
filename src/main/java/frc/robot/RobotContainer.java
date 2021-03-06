package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.PrepFeederCommand;
import frc.robot.commands.accelerator.AcceleratorBackupCommand;
import frc.robot.commands.accelerator.AcceleratorShootCommand;
import frc.robot.commands.auto.Tarmac2Ball;
import frc.robot.commands.auto.Tarmac2BallHide;
import frc.robot.commands.auto.Tarmac3Ball;
import frc.robot.commands.auto.Tarmac5Ball;
import frc.robot.commands.auto.Tarmac6Ball;
import frc.robot.commands.climber.ClimberEnableCommand;
import frc.robot.commands.drivetrain.DriveTeleopCommand;
import frc.robot.commands.feeder.FeederBackupCommand;
import frc.robot.commands.feeder.FeederIntakeCommand;
import frc.robot.commands.feeder.FeederShootCommand;
import frc.robot.commands.hood.HoodAimCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeDeployCommand;
import frc.robot.commands.intake.IntakeStowCommand;
import frc.robot.commands.shooter.ShooterAimCommand;
import frc.robot.commands.shooter.ShooterBackupCommand;
import frc.robot.commands.shooter.ShooterEjectCommand;
import frc.robot.commands.shooter.ShooterShootCommand;
import frc.robot.commands.turret.TurretAimCommand;
import frc.robot.commands.turret.TurretEjectCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
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
  private final Button shootButton = new Button(driverController::getRightTriggerButton);
  private final Button retractIntake = new Button(driverController::getSquareButton);
  private final Button backupButton = new Button(driverController::getTriangleButton);
  private final Button ejectButton = new Button(driverController::getCircleButton);
  private final Button fenderShotButton = new Button(driverController::getRightBumperButton);

  private final Button climberButtonUp =
      new Button(
          () ->
              (driverController.getPOV() == ControllerWrapper.Direction.Up)
                  || (secondaryController.getPOV() == ControllerWrapper.Direction.Up));
  private final Button climberButtonDown =
      new Button(
          () ->
              (driverController.getPOV() == ControllerWrapper.Direction.Down)
                  || (secondaryController.getPOV() == ControllerWrapper.Direction.Down));

  private final Button climberEnableButton1 = new Button(secondaryController::getLeftBumperButton);
  private final Button climberEnableButton2 = new Button(secondaryController::getRightBumperButton);
  private final Button disableSnapAround = new Button(secondaryController::getTriangleButton);
  private final Button overrideFeederButton = new Button(secondaryController::getXButton);

  private final Button gyroResetButton = new Button(driverController::getShareButton);

  // Swerve inputs
  Supplier<Double> inputX = () -> -driverController.getLeftJoystickX();
  Supplier<Double> inputY = driverController::getLeftJoystickY;
  Supplier<Double> inputRotation = () -> -driverController.getRightJoystickX();

  /** Constructs a RobotContainer */
  public RobotContainer() {
    // Zero the swerve heading
    swerve.zeroHeading();

    // Configure button bindings
    configureButtonBindings();

    // Configure default commands
    configureDefaultCommands();

    // Add autonomous command to autoChooser
    autoChooser.setDefaultOption(Autos.TARMAC_3_BALL.name(), Autos.TARMAC_3_BALL);
    autoChooser.addOption(Autos.TARMAC_2_BALL.name(), Autos.TARMAC_2_BALL);
    autoChooser.addOption(Autos.TARMAC_2_BALL_HIDE.name(), Autos.TARMAC_2_BALL_HIDE);
    autoChooser.addOption(Autos.TARMAC_3_BALL.name(), Autos.TARMAC_3_BALL);
    autoChooser.addOption(Autos.TARMAC_5_BALL.name(), Autos.TARMAC_5_BALL);
    autoChooser.addOption(Autos.TARMAC_6_BALL.name(), Autos.TARMAC_6_BALL);

    // Publish autoChooser to SmartDashboard
    SmartDashboard.putData(autoChooser);
  }

  /** Configures all default commands */
  private void configureDefaultCommands() {
    // Set the swerve default command to a DriveTeleopCommand
    swerve.setDefaultCommand(new DriveTeleopCommand(inputX, inputY, inputRotation, true, swerve));

    // Set the feeder default command to a FeederIntakeCommand
    feeder.setDefaultCommand(new FeederIntakeCommand(feeder));

    // Set the shooter default command to ShooterAimCommand
    shooter.setDefaultCommand(
        new ShooterAimCommand(
            shooter,
            () -> limelight.getCurrentTarget().hasTarget,
            () -> limelight.getCurrentTarget().range));

    // Set the accelerator default command to AcceleratorShootCommand
    accelerator.setDefaultCommand(new AcceleratorShootCommand(accelerator, shooter::getRPM));

    // Set the hood default command to a HoodAimCommand
    hood.setDefaultCommand(
        new HoodAimCommand(
            hood,
            () -> limelight.getCurrentTarget().hasTarget,
            () -> limelight.getCurrentTarget().range,
            fenderShotButton::get));

    // Set the turret default command to a TurretAimCommand
    turret.setDefaultCommand(
        new TurretAimCommand(
            turret,
            () -> limelight.getCurrentTarget().hasTarget,
            () -> limelight.getCurrentTarget().targetAngle,
            fenderShotButton::get));
  }

  /** Configures all button bindings */
  private void configureButtonBindings() {
    // When the intake button is start intaking
    intakeButton.toggleWhenPressed(
        new SequentialCommandGroup(new IntakeDeployCommand(intake), new IntakeCommand(intake)));

    // When the retract button is pressed stow the intake
    retractIntake.whenPressed(new IntakeStowCommand(intake));

    // While the shoot button is held shoot
    shootButton.whileHeld(
        new ParallelCommandGroup(
            new ShooterShootCommand(
                shooter,
                () -> limelight.getCurrentTarget().hasTarget,
                () -> limelight.getCurrentTarget().range,
                fenderShotButton::get),
            new AcceleratorShootCommand(accelerator, shooter::getRPM),
            new FeederShootCommand(feeder, shooter::atSpeed)));

    // While the backup button is held backup the shooter, accelerator, and feeder
    backupButton.whileHeld(
        new ParallelCommandGroup(
            new ShooterBackupCommand(shooter),
            new AcceleratorBackupCommand(accelerator),
            new FeederBackupCommand(feeder)));

    // While the eject button is held run eject on the turret and shooter
    ejectButton.whileHeld(
        new ParallelCommandGroup(
            new TurretEjectCommand(
                turret,
                () -> limelight.getCurrentTarget().hasTarget,
                () -> limelight.getCurrentTarget().targetAngle),
            new ShooterEjectCommand(shooter),
            new AcceleratorShootCommand(accelerator, shooter::getRPM),
            new FeederShootCommand(feeder, shooter::atSpeed)));

    // While the fender shot button is held fender shoot
    fenderShotButton.whileHeld(
        new ParallelCommandGroup(
            new ShooterShootCommand(
                shooter,
                () -> limelight.getCurrentTarget().hasTarget,
                () -> limelight.getCurrentTarget().range,
                fenderShotButton::get),
            new AcceleratorShootCommand(accelerator, shooter::getRPM),
            new FeederShootCommand(feeder, shooter::atSpeed)));

    // When the gyro reset button is pressed run an InstantCommand that zeroes the swerve heading
    gyroResetButton.whenPressed(new InstantCommand(swerve::zeroHeading));

    // When both the climber enable buttons are pressed, enable climb mode
    climberEnableButton1
        .and(climberEnableButton2)
        .whenActive(new ClimberEnableCommand(climber, turret));

    // When the override feeder button is pressed override the default feeder command
    overrideFeederButton.whenPressed(
        new InstantCommand(
            () -> feeder.setDefaultCommand(new PrepFeederCommand(feeder, accelerator))));

    // When the D-Pad is held up run the shooter up
    climberButtonUp.whenHeld(new StartEndCommand(climber::up, climber::stop));

    // When the D-Pad is held down run the shooter down
    climberButtonDown.whenHeld(new StartEndCommand(climber::down, climber::stop));

    // If disable snap around is pressed disable snap around
    disableSnapAround.whenPressed(new InstantCommand(turret::disableSnapAround));
  }

  public enum Autos {
    TARMAC_2_BALL,
    TARMAC_2_BALL_HIDE,
    TARMAC_3_BALL,
    TARMAC_5_BALL,
    TARMAC_6_BALL
  }

  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case TARMAC_2_BALL:
        return new Tarmac2Ball(swerve, accelerator, feeder, intake);
      case TARMAC_2_BALL_HIDE:
        return new Tarmac2BallHide(swerve, shooter, accelerator, feeder, intake, limelight);
      case TARMAC_3_BALL:
        return new Tarmac3Ball(swerve, accelerator, feeder, intake);
      case TARMAC_5_BALL:
        return new Tarmac5Ball(swerve, accelerator, feeder, intake);
      case TARMAC_6_BALL:
        return new Tarmac6Ball(swerve, accelerator, feeder, intake);
      default:
        return new Tarmac5Ball(swerve, accelerator, feeder, intake);
    }
  }
}
