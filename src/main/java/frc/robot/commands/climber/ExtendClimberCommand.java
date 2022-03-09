package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ExtendClimberCommand extends CommandBase {

    private enum ClimbState {
        EXTENDED,
        STOWED,
        HOOK_DEPLOYED,
        EXTENDED_NO_HOOK,
    }

    private static final int EXTENDED_POSITION = 259528;
    private static final int DEPLOY_HOOK_POSITION = EXTENDED_POSITION - 65711;
    private static final int EXTENDED_HOOK_POSITION = DEPLOY_HOOK_POSITION + 10971;

    private final Climber climber;
    private ClimbState state = ClimbState.STOWED;

    private final Timer timer;

    public ExtendClimberCommand(Climber climber) {
        this.climber = climber;
        this.timer = new Timer();
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (state == ClimbState.STOWED) {
            climber.up();
            if (climber.isAbove(EXTENDED_POSITION)) {
                state = ClimbState.EXTENDED_NO_HOOK;
                climber.stop();
            }
        } else if (state == ClimbState.EXTENDED_NO_HOOK) {
            climber.down();
            if (climber.isBelow(DEPLOY_HOOK_POSITION)) {
                state = ClimbState.HOOK_DEPLOYED;
                climber.stop();
            }
        } else if (state == ClimbState.HOOK_DEPLOYED) {
            climber.up();
            if (climber.isAbove(EXTENDED_HOOK_POSITION)) {
                state = ClimbState.EXTENDED;
                climber.stop();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return state == ClimbState.EXTENDED;
    }

}
