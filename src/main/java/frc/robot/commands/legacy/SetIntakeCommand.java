package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntakeCommand extends CommandBase {

  public enum IntakeMode {
    DEPLOY,
    RETRACT
  }

  private final IntakeMode mode;
  private final Intake intake;

  private final Timer deployTimer = new Timer();
  private final Timer retractTimer = new Timer();

  private static final double DEPLOY_TIME = 1.0; // seconds
  private static final double RETRACT_TIME = 1.0; // seconds
  private static final double RETRACT_MIN_VELOCITY = 1000; // units/100ms

  public SetIntakeCommand(Intake intake, IntakeMode mode) {
    this.intake = intake;
    this.mode = mode;

    deployTimer.stop();
    deployTimer.reset();

    retractTimer.stop();
    retractTimer.reset();

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    if (mode == IntakeMode.DEPLOY) {
      intake.coast();
      intake.deploy();

      deployTimer.stop();
      deployTimer.reset();
      deployTimer.start();
    } else if (mode == IntakeMode.RETRACT) {
      intake.retract();

      retractTimer.stop();
      retractTimer.reset();
      retractTimer.start();
    }
  }

  @Override
  public void execute() {
    if (deployTimer.hasElapsed(DEPLOY_TIME)) {
      intake.stopRetraction();
    }

    if (retractTimer.hasElapsed(RETRACT_TIME)) {
      if (intake.getRetractVelocity() <= RETRACT_MIN_VELOCITY) {
        intake.stopRetraction();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return deployTimer.hasElapsed(DEPLOY_TIME)
        || (retractTimer.hasElapsed(RETRACT_TIME)
            && intake.getRetractVelocity() <= RETRACT_MIN_VELOCITY);
  }

  @Override
  public void end(boolean interrupted) {
    intake.brake();
    intake.stopRetraction();
  }
}
