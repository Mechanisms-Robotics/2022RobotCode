package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

public class AimSwerveDrive extends DriveTeleopCommand {
    private final PIDController aimPID = new PIDController(0.06, 0.00015, 0.00, Constants.loopTime);
    // Last Working PID Values
    // kP: 0.07 //
    private final PhotonCamera camera;
    private final NetworkTable photonNetworkTable;

    private final double ANGLE_FUDGE_FACTOR = -3.0;

    public AimSwerveDrive(
            Supplier<Double> driverX,
            Supplier<Double> driverY,
            Supplier<Double> rotation,
            Swerve swerve,
            PhotonCamera camera) {
        super(driverX, driverY, rotation, swerve);
        this.camera = camera;
        aimPID.setTolerance(0.75); // Degrees

        // TODO: Figure out bug with usenig camera results for these values
        photonNetworkTable =
                NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("limelight");
    }

    @Override
    public void execute() {
        var cameraResults = camera.getLatestResult();
        if (cameraResults.hasTargets()) {
            final double yaw =
                    photonNetworkTable.getEntry("targetYaw").getDouble(0.0) + ANGLE_FUDGE_FACTOR;
            double pidOutput = aimPID.calculate(yaw);
            pidOutput = MathUtil.clamp(pidOutput, -Math.PI, Math.PI);
            super.driveRotationVelocityMode(
                    deadband(vxSupplier.get()), deadband(vySupplier.get()), pidOutput);
        } else {
            super.execute();
        }
    }
}