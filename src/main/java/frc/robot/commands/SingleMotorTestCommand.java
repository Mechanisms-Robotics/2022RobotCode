package frc.robot.commands;

import frc.robot.subsystems.SingleMotorTestSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SingleMotorTestCommand extends CommandBase {

    private final SingleMotorTestSubsystem driveSubsystem;
    private double driveValue;

    public SingleMotorTestCommand(SingleMotorTestSubsystem subsystem, double value) {
        driveSubsystem = subsystem;
        this.driveValue = value;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.set(driveValue);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
