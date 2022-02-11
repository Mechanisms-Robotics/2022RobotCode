package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.BackupCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SwerveCalibrationCommand;
import frc.robot.commands.auto.Basic1Ball;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ControllerWrapper;
import java.util.function.Supplier;

public class RobotContainer {

  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();
  private final Feeder feeder = new Feeder();
  private final Accelerator accelerator = new Accelerator();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();

  private final ControllerWrapper controllerWrapper = new ControllerWrapper(0);

  private final Button intakeButton = new Button(controllerWrapper::getLeftTriggerButton);
  private final Button outtakeButton = new Button(controllerWrapper::getTriangleButton);
  private final Button shootButton = new Button(controllerWrapper::getRightTriggerButton);

  Supplier<Double> inputX = () -> -controllerWrapper.getLeftJoystickX(),
      inputY = () -> -controllerWrapper.getLeftJoystickY(),
      rotation = () -> -controllerWrapper.getRightJoystickX();

  public RobotContainer() {
    swerve.zeroHeading();
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    // swerve.setDefaultCommand(new DriveTeleopCommand(inputX, inputY, rotation, true, swerve));
  }

  private void configureButtonBindings() {
    intakeButton.toggleWhenPressed(
        new StartEndCommand(
            () -> {
              new IntakeCommand(intake, feeder).schedule();
            },
            () -> {
              new BackupCommand(accelerator, feeder).schedule();
            }));
    outtakeButton.whenHeld(new OuttakeCommand(intake, feeder, accelerator));
    shootButton.toggleWhenPressed(new ShootCommand(shooter, accelerator, feeder));
  }

  public Command getAutonomousCommand() {
    return new Basic1Ball(swerve);
  }

  public void runDisabledCalibration() {
    CommandScheduler.getInstance().schedule(new SwerveCalibrationCommand(swerve));
  }
}
