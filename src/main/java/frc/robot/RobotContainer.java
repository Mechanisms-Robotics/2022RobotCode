package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ControllerWrapper;

public class RobotContainer {

    private final Swerve swerve = new Swerve();

    private final ControllerWrapper controllerWrapper = new ControllerWrapper(0);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                new DriveTeleopCommand(
                        () -> -controllerWrapper.getLeftJoystickX(),
                        () -> -controllerWrapper.getLeftJoystickY(),
                        () -> -controllerWrapper.getRightJoystickX(),
                        true,
                        swerve));
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
