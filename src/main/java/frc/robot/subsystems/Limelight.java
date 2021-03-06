package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

/** This class contains all the code responsible for tracking the goal using the Limelight. */
public class Limelight extends SubsystemBase {
  // Instance of limelight
  private final PhotonCamera limelight;

  // Constants
  private static final double TARGET_HEIGHT = 2.64; // meters
  private static final double CAMERA_HEIGHT = 1.024; // meters
  private static final double CAMERA_PITCH = Math.toRadians(60.0); // radians

  public static class TargetData {
    public boolean hasTarget = false;
    public double range;
    public double targetAngle;
  }

  private final TargetData currentTarget = new TargetData();

  /** Constructs a GoalTracker */
  public Limelight() {
    // Instantiate the limelight and limelight network table
    this.limelight = new PhotonCamera("limelight");

    // Turn on the limelight's LEDs
    turnOnLEDs();
  }

  @Override
  public void periodic() {
    var limelightResults = this.limelight.getLatestResult();
    if (limelightResults.hasTargets()) {
      var bestTarget = limelightResults.getBestTarget();
      currentTarget.hasTarget = true;
      currentTarget.targetAngle = -bestTarget.getYaw();
      currentTarget.range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT,
              TARGET_HEIGHT,
              CAMERA_PITCH,
              Units.degreesToRadians(bestTarget.getPitch()));
      SmartDashboard.putBoolean("Has Target", currentTarget.hasTarget);
      SmartDashboard.putNumber("Target Angle", currentTarget.targetAngle);
      SmartDashboard.putNumber("Target Range", currentTarget.range);
    } else {
      currentTarget.hasTarget = false;
      SmartDashboard.putBoolean("Has Target", currentTarget.hasTarget);
    }
  }

  public TargetData getCurrentTarget() {
    return currentTarget;
  }

  /** Turns on the limelight's LEDs */
  public void turnOnLEDs() {
    this.limelight.setLED(VisionLEDMode.kOn);
  }

  /** Turns off the limelight's LEDs */
  public void turnOffLEDs() {
    this.limelight.setLED(VisionLEDMode.kOff);
  }
}
