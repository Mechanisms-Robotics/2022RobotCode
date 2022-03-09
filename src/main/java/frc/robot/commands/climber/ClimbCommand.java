package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbCommand extends CommandBase {

    private static final int CLIMBED_POSITION = 0;

    private final Climber climber;

    public ClimbCommand(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void execute() {
        if (climber.isAbove(CLIMBED_POSITION)) {
            climber.down();
        } else {
            climber.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return climber.isBelow(CLIMBED_POSITION);
    }

}
