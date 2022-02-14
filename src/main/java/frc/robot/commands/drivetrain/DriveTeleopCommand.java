package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.Optional;
import java.util.function.Supplier;

/** Command to drive the swerve in teleop. Supplied left joystick x and y, and right joystick x. */
public class DriveTeleopCommand extends CommandBase {

  protected static final double MAX_TRANSLATIONAL_VELOCITY_RATE = 10; // m/s per second
  protected static final double MAX_ROTATION_VELOCITY_RATE = 4 * Math.PI; // rads/s per second

  protected static final double TRANSLATION_CURVE_STRENGTH = 10000.0;
  protected static final double ROTATION_CURVE_STRENGTH = 10.0; // 10.0 makes it effectively linear.

  private static final double DEADBAND = 0.15;

  protected final Swerve swerve;

  protected final Supplier<Double> vxSupplier;
  protected final Supplier<Double> vySupplier;
  protected final Supplier<Double> vrxSupplier;
  protected final Optional<Supplier<Double>> vrySupplier;
  private final SlewRateLimiter vxRateLimiter;
  private final SlewRateLimiter vyRateLimiter;
  private final SlewRateLimiter vrRateLimiter;
  protected final boolean fieldOriented;

  private DriveTeleopCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRotationX,
      Optional<Supplier<Double>> driverRotationY,
      boolean fieldOriented,
      Swerve swerve) {
    vxSupplier = driverX;
    vySupplier = driverY;
    vrxSupplier = driverRotationX;
    vrySupplier = driverRotationY;

    this.fieldOriented = fieldOriented;

    this.swerve = swerve;
    addRequirements(this.swerve);

    vxRateLimiter = new SlewRateLimiter(MAX_TRANSLATIONAL_VELOCITY_RATE);
    vyRateLimiter = new SlewRateLimiter(MAX_TRANSLATIONAL_VELOCITY_RATE);
    vrRateLimiter = new SlewRateLimiter(MAX_ROTATION_VELOCITY_RATE);
  }

  /**
   * Constructs the DriveTeleopCommand.
   *
   * @param driverX Left joystick x, which acts as desired x translation of the swerve drive
   * @param driverY Left joystick y, which acts as desired y translation of the swerve drive
   * @param driverRotation Right joystick x, which acts as desired rotation of the swerve drive
   * @param fieldOriented Whether driving is field oriented
   * @param swerve Instance of Swerve
   */
  public DriveTeleopCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRotation,
      boolean fieldOriented,
      Swerve swerve) {
    this(driverX, driverY, driverRotation, Optional.empty(), fieldOriented, swerve);
  }

  /**
   * Constructs the DriveTeleopCommand.
   *
   * @param driverX Left joystick x, which acts as desired x translation of the swerve drive
   * @param driverY Left joystick y, which acts as desired y translation of the swerve drive
   * @param driverRotation Right joystick x, which acts as desired rotation of the swerve drive
   * @param swerve Instance of Swerve
   */
  public DriveTeleopCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRotation,
      Swerve swerve) {
    this(driverX, driverY, driverRotation, true, swerve);
  }

  private Rotation2d currentRotationCommand;

  @Override
  public void execute() {
    final double translationX = deadband(vxSupplier.get());
    final double translationY = deadband(vySupplier.get());
    final double rotationX = deadband(vrxSupplier.get());
    /*
    if (vrySupplier.isPresent()) {
      final double rotationY = deadband(vrySupplier.get().get());
      if (Math.abs(rotationX) > 0 || Math.abs(rotationY) > 0) {
        currentRotationCommand = new Rotation2d(rotationX, rotationY);
      }
      driveRotationPositionMode(translationX, translationY, currentRotationCommand);
    } else {
      System.out.println("SPAM");
      driveRotationVelocityMode(translationX, translationX, rotationX);
    }
    */
    driveRotationVelocityMode(translationX, translationY, rotationX);
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  protected void driveRotationVelocityMode(double dx, double dy, double dr) {
    // Scale the tranlational input
    Translation2d translation = scaleTranslationInput(new Translation2d(dx, dy));

    // Apply Ramp Rates
    dx = vxRateLimiter.calculate(translation.getY() * Swerve.maxVelocity);
    dy = vyRateLimiter.calculate(-translation.getX() * Swerve.maxVelocity);
    dr = vrRateLimiter.calculate(dr * Swerve.maxRotationalVelocity);

    swerve.drive(dx, dy, dr, fieldOriented);
  }

  protected void driveRotationPositionMode(double dx, double dy, Rotation2d rotation) {
    Translation2d translation = scaleTranslationInput(new Translation2d(dx, dy));
    swerve.drive(translation.getX(), translation.getY(), rotation);
  }

  private Translation2d scaleTranslationInput(Translation2d input) {
    double mag = input.getNorm();
    final double scale = 1.35;
    mag = Math.pow(mag, scale);
    final Rotation2d rotation = new Rotation2d(input.getX(), input.getY());
    return new Translation2d(rotation.getCos() * mag, rotation.getSin() * mag);
  }

  protected static double deadband(double input) {
    return Math.abs(input) > DEADBAND ? input : 0;
  }
}
