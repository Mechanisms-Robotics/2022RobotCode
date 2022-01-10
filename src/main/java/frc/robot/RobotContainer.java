
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    public RobotContainer() {
        configureButtonBindings();
        CommandScheduler.getInstance().schedule(new DriveCommand(driveSubsystem));
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
