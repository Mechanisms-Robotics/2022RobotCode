
package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    private final DriveSubsystem m_exampleSubsystem = new DriveSubsystem();

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}
