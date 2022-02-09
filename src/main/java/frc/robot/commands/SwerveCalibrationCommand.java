package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/** Wait 7 sec then calibrate the swerve modules; */
public class SwerveCalibrationCommand extends CommandBase {

  private final Swerve swerve;
  private static final double CALIBRATION_WAIT_TIME = 7.0; // sec
  private final Timer timer = new Timer();

  public SwerveCalibrationCommand(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(CALIBRATION_WAIT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    // Only reset the modules if this command wasn't interrupted (i.e. the timer ran out)
    if (!interrupted) {
      swerve.calibrateModules();
      System.out.println("Recalibrating Swerve Modules");
    } else {
      DriverStation.reportWarning("Swerve calibration canceled.", false);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
