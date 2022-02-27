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

  // Max velocity and acceleration rates
  protected static final double MAX_TRANSLATIONAL_VELOCITY_RATE = 4.0; // m/s per second
  protected static final double MAX_ROTATION_VELOCITY_RATE = 2 * Math.PI; // rads/s per second

  // Joystick deadband
  private static final double DEADBAND = 0.1; // joystick percentage

  // Instance of swerve
  protected final Swerve swerve;

  // Suppliers for translation x and y, and rotation x
  protected final Supplier<Double> vxSupplier;
  protected final Supplier<Double> vySupplier;
  protected final Supplier<Double> vrxSupplier;

  // Optional supplier for rotation y
  protected final Optional<Supplier<Double>> vrySupplier;

  // Rate limiters for translation and rotation suppliers
  private final SlewRateLimiter vxRateLimiter;
  private final SlewRateLimiter vyRateLimiter;
  private final SlewRateLimiter vrRateLimiter;

  // Whether we are fieldOriented or not
  protected final boolean fieldOriented;

  /**
   * Constructs the DriveTeleopCommand
   *
   * @param driverX Left joystick x, which acts as desired x translation of the swerve drive
   * @param driverY Left joystick y, which acts as desired y translation of the swerve drive
   * @param driverRotationX Right joystick x, which acts as desired rotation of the swerve drive
   * @param driverRotationY Right joystick y, which acts as desired rotation of the swerve drive
   * @param fieldOriented Whether driving is field oriented
   * @param swerve Instance of Swerve
   */
  private DriveTeleopCommand(
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRotationX,
      Optional<Supplier<Double>> driverRotationY,
      boolean fieldOriented,
      Swerve swerve) {
    // Set the suppliers
    vxSupplier = driverX;
    vySupplier = driverY;
    vrxSupplier = driverRotationX;
    vrySupplier = driverRotationY;

    // Set field oriented
    this.fieldOriented = fieldOriented;

    // Set the swerve and add it as a requirement
    this.swerve = swerve;
    addRequirements(this.swerve);

    // Instantiate rate limiters
    vxRateLimiter = new SlewRateLimiter(MAX_TRANSLATIONAL_VELOCITY_RATE);
    vyRateLimiter = new SlewRateLimiter(MAX_TRANSLATIONAL_VELOCITY_RATE);
    vrRateLimiter = new SlewRateLimiter(MAX_ROTATION_VELOCITY_RATE);
  }

  /**
   * Constructs the DriveTeleopCommand
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
    // Calls constructor with an empty driverRotationY
    this(driverX, driverY, driverRotation, Optional.empty(), fieldOriented, swerve);
  }

  /**
   * Constructs the DriveTeleopCommand
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
    // Calls constructor with fieldOriented set to true
    this(driverX, driverY, driverRotation, true, swerve);
  }

  // private Rotation2d currentRotationCommand;

  /**
   * Gets the driver translation and rotation input, deadbands it, then calls
   * driveRotationVelocityMode
   */
  @Override
  public void execute() {
    // Get the driver translation and rotation inputs then deadband them
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
    // Drive rotation velocity mode
    driveRotationVelocityMode(translationX, translationY, rotationX);
  }

  /**
   * Checks if this command is finished
   *
   * @return Is this command finished
   */
  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }

  /**
   * Stops the swerve drive
   *
   * @param interrupted Whether this command was interrupted or not
   */
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  /**
   * Drives the swerve drive with a desired x, y, and rotation.
   *
   * @param dx Desired x translation
   * @param dy Desired y translation
   * @param dr Desired rotation
   */
  protected void driveRotationVelocityMode(double dx, double dy, double dr) {
    // Scale the translational input
    Translation2d translation = scaleTranslationInput(new Translation2d(dx, dy));

    // Apply Ramp Rates
    dx = vxRateLimiter.calculate(translation.getY() * Swerve.maxVelocity);
    dy = vyRateLimiter.calculate(-translation.getX() * Swerve.maxVelocity);
    dr = vrRateLimiter.calculate(dr * Swerve.maxRotationalVelocity);

    // Drive the swerve
    swerve.drive(dx, dy, dr, fieldOriented);
  }

  /**
   * Drives the swerve drive with a desired x and y position, and rotation
   *
   * @param dx Desired x position
   * @param dy Desired y position
   * @param rotation Desired rotation
   */
  protected void driveRotationPositionMode(double dx, double dy, Rotation2d rotation) {
    // Scale translational inputs
    Translation2d translation = scaleTranslationInput(new Translation2d(dx, dy));

    // Drive the swerve
    swerve.drive(translation.getX(), translation.getY(), rotation);
  }

  /**
   * Scales translational inputs to an exponential curve
   *
   * @param input Translational input
   * @return Scaled translational input
   */
  private Translation2d scaleTranslationInput(Translation2d input) {
    // Get the magnitude of the translation
    double mag = input.getNorm();

    // Scale it to an exponential
    final double scale = 1.35;
    mag = Math.pow(mag, scale);

    // Multiply the translational input with the exponential, and return it
    final Rotation2d rotation = new Rotation2d(input.getX(), input.getY());
    return new Translation2d(rotation.getCos() * mag, rotation.getSin() * mag);
  }

  /**
   * Checks if the input is with the deadband, if so zeros it
   *
   * @param input The input to deadband
   * @return The dead-banded input
   */
  protected static double deadband(double input) {
    return Math.abs(input) > DEADBAND ? input : 0;
  }
}
