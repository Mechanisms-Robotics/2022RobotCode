package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

/** Enables the climber and locks the turret */
public class ClimberEnableCommand extends CommandBase {

  // Instance of Climber and Turret
  private final Climber climber;
  private final Turret turret;

  // Timer for clearing the wirechain of the climber
  private final Timer wireChainClearTimer;

  // Time to wait for the wirechain to clear the climber
  private static final double WIRE_CHAIN_CLEAR_TIME = 1.0; // seconds

  public ClimberEnableCommand(Climber climber, Turret turret) {
    // Set climber and turret
    this.climber = climber;
    this.turret = turret;

    // Set wire chain clear timer
    wireChainClearTimer = new Timer();
    wireChainClearTimer.stop();
    wireChainClearTimer.reset();
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    turret.snapTo(Math.toRadians(0.0));

    wireChainClearTimer.reset();
    wireChainClearTimer.start();
  }

  /**
   * Returns whether the command is finished or not
   *
   * @return Whether the command is finished or not
   */
  @Override
  public boolean isFinished() {
    return wireChainClearTimer.hasElapsed(WIRE_CHAIN_CLEAR_TIME);
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    turret.snapTo(Math.toRadians(-90.0));
    turret.lockTurret();
    climber.enableClimber();
  }
}
