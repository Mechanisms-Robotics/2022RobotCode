package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class DeployIntakeCommand extends CommandBase {

    private static final double UP_POWER = 0.50;
    private static final double DOWN_POWER = -0.25;
    private static final double DEPLOY_WAIT_TIME = 0.25;

    private static final int DEPLOYED_POSITION = 20523;
    private static final int STOWED_POSITION = 100;

    private final Climber climber;

    private boolean deployed = false;
    private boolean finished = false;

    private final Timer timer;

    public DeployIntakeCommand(Climber climber) {
        this.climber = climber;
        this.timer = new Timer();
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (!deployed) {
            climber.up();
            if (climber.isAbove(DEPLOYED_POSITION)) {
                climber.stop();
                deployed = true;
                timer.start();
                timer.reset();
            }
        } else {
            if (timer.hasElapsed(DEPLOY_WAIT_TIME)) {
                climber.down();
                if (climber.isBelow(STOWED_POSITION)) {
                    climber.stop();
                    finished = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
