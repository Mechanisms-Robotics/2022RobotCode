package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/** Deploys the intake */
public class IntakeDeployCommand extends CommandBase {

  // Instance of intake
  private final Intake intake;

  // This timer is used to wait for the motor to start moving before trying to detect stalling
  private final Timer stallTimer = new Timer();

  // The amount of time to wait before trying to detect stalling
  private static final double STALL_WAIT_TIME = 0.5; // seconds
  // The velocity that if dropped below, indicates stalling
  private static final double STALL_VELOCITY = 1000; // ticks/100ms

  /**
   * Constructs an IntakeDeployCommand
   *
   * @param intake An instance of Intake
   */
  public IntakeDeployCommand(Intake intake) {
    // Set intake
    this.intake = intake;

    // Stop and reset the stall timer
    stallTimer.stop();
    stallTimer.reset();

    // Add the intake as a requirement
    addRequirements(intake);
  }

  /** Runs when the command is first started */
  @Override
  public void initialize() {
    // Check if the intake is stowed
    if (intake.isStowed()) {
      // If the intake is stowed start deploying it
      intake.coast();
      intake.deploy();

      // Reset and start the stall timer
      stallTimer.reset();
      stallTimer.start();
    } else {
      // If the intake is already deployed set the intake state to deployed
      intake.setDeployed(true);
    }
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Check if STALL_WAIT_TIME has elapsed
    if (stallTimer.hasElapsed(STALL_WAIT_TIME)) {
      // If the velocity has dropped below STALL_VELOCITY the motor has started stalling
      if (intake.getRetractVelocity() <= STALL_VELOCITY) {
        // Set the intake state to deployed
        intake.setDeployed(true);
      }
    }
  }

  /**
   * Returns whether the command is finished or not
   *
   * @return Whether the command is finished or not
   */
  @Override
  public boolean isFinished() {
    // Return true if the intake is deployed
    return intake.isDeployed();
  }

  /**
   * Runs when the command ends
   *
   * @param interrupted Whether the command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the intake deploy motor
    intake.stopDeploying();
  }
}
