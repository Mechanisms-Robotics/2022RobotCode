package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.accelerator.AcceleratorShootCommand;
import frc.robot.commands.auto.Tarmac2Ball;
import frc.robot.commands.auto.Tarmac2BallHide;
import frc.robot.commands.auto.Tarmac3Ball;
import frc.robot.commands.auto.Tarmac5Ball;
import frc.robot.commands.auto.Tarmac6Ball;
import frc.robot.commands.drivetrain.DriveTeleopCommand;
import frc.robot.commands.feeder.FeederIntakeCommand;
import frc.robot.commands.feeder.FeederShootCommand;
import frc.robot.commands.hood.HoodAimCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeDeployCommand;
import frc.robot.commands.intake.IntakeStowCommand;
import frc.robot.commands.shooter.ShooterShootCommand;
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

  private final Button climberButtonUp =
      new Button(() -> secondaryController.getPOV() == ControllerWrapper.Direction.Up);
  private final Button climberButtonDown =
      new Button(() -> secondaryController.getPOV() == ControllerWrapper.Direction.Down);

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

    // Set the shooter default command to ShooterAimCommand
    // shooter.setDefaultCommand(new ShooterAimCommand(shooter, () ->
    // limelight.getCurrentTarget().hasTarget, () -> limelight.getCurrentTarget().range));

    // Set the hood default command to a HoodAimCommand
    hood.setDefaultCommand(
        new HoodAimCommand(
            hood,
            () -> limelight.getCurrentTarget().hasTarget,
            () -> limelight.getCurrentTarget().range));

    // Set the turret default command to a TurretAimCommand
    turret.setDefaultCommand(
        new TurretAimCommand(
            turret,
            () -> limelight.getCurrentTarget().hasTarget,
            () -> limelight.getCurrentTarget().targetAngle));
  }

  /** Configures all button bindings */
  private void configureButtonBindings() {
    // When the intake button is start intaking
    intakeButton.toggleWhenPressed(
        new SequentialCommandGroup(
            new IntakeDeployCommand(intake),
            new ParallelCommandGroup(new IntakeCommand(intake), new FeederIntakeCommand(feeder))));

    // When the retract button is pressed stow the intake
    retractIntake.whenPressed(new IntakeStowCommand(intake));

    // When the D-Pad is held up run the shooter up
    climberButtonUp.whenHeld(new StartEndCommand(climber::up, climber::stop));

    // When the D-Pad is held down run the shooter down
    climberButtonDown.whenHeld(new StartEndCommand(climber::down, climber::stop));

    // While the shoot button is held shoot
    shootButton.whileHeld(
        new ParallelCommandGroup(
            new FeederShootCommand(feeder),
            new AcceleratorShootCommand(accelerator),s
            new ShooterShootCommand(
                shooter,
                () -> limelight.getCurrentTarget().hasTarget,
                () -> limelight.getCurrentTarget().range)));

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

  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case TARMAC_2_BALL:
        return new Tarmac2Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      case TARMAC_2_BALL_HIDE:
        return new Tarmac2BallHide(
            swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      case TARMAC_3_BALL:
        return new Tarmac3Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      case TARMAC_5_BALL:
        return new Tarmac5Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      case TARMAC_6_BALL:
        return new Tarmac6Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
      default:
        return new Tarmac5Ball(swerve, shooter, turret, hood, accelerator, feeder, intake, climber);
    }
  }
}
