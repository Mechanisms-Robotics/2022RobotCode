
package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    private final Swerve swerve = new Swerve();

    private final PS4Controller driverController = new PS4Controller(0);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                new DriveTeleopCommand(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        () -> -driverController.getRightX(),
                        true,
                        swerve));
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
