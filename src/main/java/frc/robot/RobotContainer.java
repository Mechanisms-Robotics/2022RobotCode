package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.auto.Basic1Ball;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ControllerWrapper;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

public class RobotContainer {

  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();

  private final ControllerWrapper controllerWrapper = new ControllerWrapper(0);

  private final PhotonCamera camera = new PhotonCamera("limelight");

  private final Button aimButton = new Button(controllerWrapper::getRightBumperButton);
  private final Button intakeButton = new Button(controllerWrapper::getLeftTriggerButton);
  private final Button stopIntakeButton = new Button(controllerWrapper::getRightTriggerButton);

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
    aimButton.toggleWhenPressed(new AimCommand(inputX, inputY, rotation, hood, swerve, camera));
    intakeButton.toggleWhenPressed(new IntakeCommand(controllerWrapper::getLeftTriggerButton, intake));
    stopIntakeButton.toggleWhenPressed(new InstantCommand(intake::stop));
  }

  public Command getAutonomousCommand() {
    return new Basic1Ball(swerve);
  }
}
