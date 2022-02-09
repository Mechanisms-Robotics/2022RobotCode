package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.auto.Basic1Ball;
import frc.robot.commands.drivetrain.DriveTeleopCommand;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ControllerWrapper;
import frc.robot.util.ControllerWrapper.Direction;
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
  private final Button shootButton = new Button(controllerWrapper::getRightTriggerButton);

  private final Button feederButton = new Button(controllerWrapper::getLeftBumperButton);
  private final Button acceleratorButton = new Button(controllerWrapper::getRightBumperButton);
  private final Button flywheelButton = new Button(controllerWrapper::getXButton);

  Supplier<Double> inputX = () -> -controllerWrapper.getLeftJoystickX(),
      inputY = () -> -controllerWrapper.getLeftJoystickY(),
      rotation = () -> -controllerWrapper.getRightJoystickX();

  public RobotContainer() {
    swerve.zeroHeading();
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    swerve.setDefaultCommand(new DriveTeleopCommand(inputX, inputY, rotation, true, swerve));
  }

  private void configureButtonBindings() {
    intakeButton.toggleWhenPressed(new StartEndCommand(intake::intake, intake::stop, intake));
    shootButton.toggleWhenPressed(new ShootCommand(feeder, shooter, accelerator));

    feederButton.toggleWhenPressed(new StartEndCommand(feeder::feed, feeder::stop, feeder));
    acceleratorButton.toggleWhenPressed(new StartEndCommand(accelerator::spinup, accelerator::stop));
    flywheelButton.toggleWhenPressed(new StartEndCommand(shooter::shoot, shooter::stop);
  }

  public Command getAutonomousCommand() {
    return new Basic1Ball(swerve);
  }
}
