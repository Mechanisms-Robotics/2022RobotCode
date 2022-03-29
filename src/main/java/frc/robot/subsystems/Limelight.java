package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

/** This class contains all the code responsible for tracking the goal using the Limelight. */
public class Limelight extends SubsystemBase {

  private static final double NETWORK_LATENCY = 0.0; // ms

  // Instance of limelight
  private final PhotonCamera limelight;

  // Logging
  private final DataLog log = DataLogManager.getLog();
  private final BooleanLogEntry hasTargetLog = new BooleanLogEntry(log, "limelight/hasTarget");
  private final DoubleLogEntry rangeLog = new DoubleLogEntry(log, "limelight/range");
  private final DoubleLogEntry angleLog = new DoubleLogEntry(log, "limelight/angle");
  private final DoubleLogEntry latencyLog = new DoubleLogEntry(log, "limelight/latency");
  private final StringLogEntry eventLog = new StringLogEntry(log, "limelight/events");

  // Constants
  private static final double TARGET_HEIGHT = 2.64; // meters
  private static final double CAMERA_HEIGHT = 1.024; // meters
  private static final double CAMERA_PITCH = Math.toRadians(60.0); // radians

  public static class TargetData {
    public boolean hasTarget = false;
    public double range;
    public double targetAngle;
    public double camraLatency;
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
      eventLog.append("Got photon vision target");
      var bestTarget = limelightResults.getBestTarget();
      currentTarget.hasTarget = true;
      currentTarget.targetAngle = -bestTarget.getYaw();
      currentTarget.range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT,
              TARGET_HEIGHT,
              CAMERA_PITCH,
              Units.degreesToRadians(bestTarget.getPitch()));
      currentTarget.camraLatency = limelightResults.getLatencyMillis() + NETWORK_LATENCY;
      rangeLog.append(currentTarget.range);
      angleLog.append(currentTarget.targetAngle);
      latencyLog.append(currentTarget.camraLatency);
    } else {
      currentTarget.hasTarget = false;
    }
    hasTargetLog.append(currentTarget.hasTarget);
  }

  public TargetData getCurrentTarget() {
    return currentTarget;
  }

  /** Turns on the limelight's LEDs */
  public void turnOnLEDs() {
    eventLog.append("Sent LED On signal to limelight.");
    this.limelight.setLED(VisionLEDMode.kOn);
  }

  /** Turns off the limelight's LEDs */
  public void turnOffLEDs() {
    eventLog.append("Sent LED Off signal to limelight");
    this.limelight.setLED(VisionLEDMode.kOff);
  }
}
