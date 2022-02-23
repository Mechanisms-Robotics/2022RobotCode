package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/** This command will aim the drivetrain at a target using the Limelight */
public class AimSwerveDrive extends DriveTeleopCommand {
  // PID for aiming
  private final PIDController aimPID = new PIDController(0.06, 0.00015, 0.00, Constants.loopTime);
  // Limelight
  private final PhotonCamera camera;
  // Network Tables
  private final NetworkTable photonNetworkTable;

  // This is a fudge factor so we can adjust the aiming to the left or right
  private final double ANGLE_FUDGE_FACTOR = 0.0;

  /**
   * Constructs an AimSwerveDrive command
   *
   * @param driverX The x axis input from the driver controller
   * @param driverY The y axis input from the driver controller
   * @param rotation The rotation input from the driver controller
   * @param swerve An instance of the Swerve
   * @param camera An instance of the PhotonCamera (Limelight)
   */
  public AimSwerveDrive(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> rotation,
      Swerve swerve,
      PhotonCamera camera) {
    // Call the DriveTeleopCommand constructor with these parameters
    super(driverX, driverY, rotation, swerve);

    // Set the camera variable
    this.camera = camera;

    // Set the aiming PID tolerance
    aimPID.setTolerance(0.75); // Degrees

    // TODO: Figure out bug with using camera results for these values
    // Gets the limelight table from the NetworkTables
    photonNetworkTable =
        NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("limelight");
  }

  /**
   * Checks if the Limelight has a target, if it does aim the swerve but allow the driver to still
   * control translation. If there is no target allow driver to control translation and rotation.
   */
  @Override
  public void execute() {
    // Pull the latest target from the Limelight
    var cameraResults = camera.getLatestResult();

    // Check if the Limelight has a target
    if (cameraResults.hasTargets()) {
      // Get the angle to the target
      final double yaw =
          photonNetworkTable.getEntry("targetYaw").getDouble(0.0) + ANGLE_FUDGE_FACTOR;

      // Calculate the PID output
      double pidOutput = aimPID.calculate(yaw);

      // Clamp the PID output between -π and π
      pidOutput = MathUtil.clamp(pidOutput, -Math.PI, Math.PI);

      // Drive with the driver's translation inputs but with the PIDs rotation output
      super.driveRotationVelocityMode(
          deadband(vxSupplier.get()), deadband(vySupplier.get()), pidOutput);
    } else {
      // Drive with the driver's translation and rotation inputs
      super.execute();
    }
  }
}
