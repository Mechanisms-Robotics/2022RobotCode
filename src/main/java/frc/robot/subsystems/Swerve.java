package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HeadingController;
import frc.robot.util.TrajectoryController;

/** The base swerve drive class, controls all swerve modules in coordination. */
public class Swerve extends SubsystemBase {

  public static final double maxVelocity = 4.5; // m / s
  public static final double maxRotationalVelocity = 1.5 * Math.PI; // rads/s

  // The center of the robot is the origin point fo=/\r all locations
  private static final double driveBaseWidth = 0.5969; // m
  private static final double driveBaseLength = 0.5969; // m

  private static final Translation2d flModuleLocation =
      new Translation2d(driveBaseWidth / 2.0, driveBaseLength / 2.0);
  private static final Translation2d frModuleLocation =
      new Translation2d(driveBaseWidth / 2.0, -driveBaseLength / 2.0);
  private static final Translation2d blModuleLocation =
      new Translation2d(-driveBaseWidth / 2.0, driveBaseLength / 2.0);
  private static final Translation2d brModuleLocation =
      new Translation2d(-driveBaseWidth / 2.0, -driveBaseLength / 2.0);

  private static final int flWheelMotorID = 12;
  private static final int flSteerMotorID = 13;
  private static final int flSteerEncoderID = 12;
  private static final int frWheelMotorID = 14;
  private static final int frSteerMotorID = 15;
  private static final int frSteerEncoderID = 14;
  private static final int blWheelMotorID = 10;
  private static final int blSteerMotorID = 11;
  private static final int blSteerEncoderID = 10;
  private static final int brWheelMotorID = 16;
  private static final int brSteerMotorID = 17;
  private static final int brSteerEncoderID = 16;

  private static final double flAngleOffset = -238.096; // -238.096
  private static final double frAngleOffset = -332.402; // -335.479
  private static final double blAngleOffset = -172.441; // -82.090
  private static final double brAngleOffset = -77.432; // -75.059;

  private static final int gyroID = 2;

  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          flModuleLocation, frModuleLocation, blModuleLocation, brModuleLocation);

  private final SwerveDriveOdometry poseEstimator;

  private final SwerveModule flModule =
      Mk4SwerveModuleHelper.createFalcon500(
          Mk4SwerveModuleHelper.GearRatio.L2,
          flWheelMotorID,
          flSteerMotorID,
          flSteerEncoderID,
          Math.toRadians(flAngleOffset));

  private final SwerveModule frModule =
      Mk4SwerveModuleHelper.createFalcon500(
          Mk4SwerveModuleHelper.GearRatio.L2,
          frWheelMotorID,
          frSteerMotorID,
          frSteerEncoderID,
          Math.toRadians(frAngleOffset));

  private final SwerveModule blModule =
      Mk4SwerveModuleHelper.createFalcon500(
          Mk4SwerveModuleHelper.GearRatio.L2,
          blWheelMotorID,
          blSteerMotorID,
          blSteerEncoderID,
          Math.toRadians(blAngleOffset));

  private final SwerveModule brModule =
      Mk4SwerveModuleHelper.createFalcon500(
          Mk4SwerveModuleHelper.GearRatio.L2,
          brWheelMotorID,
          brSteerMotorID,
          brSteerEncoderID,
          Math.toRadians(brAngleOffset));

  public final PigeonIMU gyro = new PigeonIMU(gyroID);

  private final HeadingController headingController =
      new HeadingController(
          0.005, // Stabilization kP
          0.0, // Stabilization kD
          1.75, // Lock kP
          0.0, // Lock kI
          0.0, // Lock kD
          2.0, // Turn in place kP
          0.0, // Turn in place kI
          0.0 // Turn in place kD
          );
  private final TrajectoryController trajectoryController = new TrajectoryController(kinematics);
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  private Rotation2d gyroAngleAdjustment = Rotation2d.fromDegrees(0.0);

  /** Constructs the Swerve subsystem. */
  public Swerve() {
    poseEstimator = new SwerveDriveOdometry(kinematics, getHeading(), new Pose2d());

    gyro.setFusedHeading(0.0);
    this.register();
    this.setName("Swerve Drive");
  }

  @Override
  public void periodic() {
    poseEstimator.update(
        getHeading(),
        flModule.getState(),
        frModule.getState(),
        blModule.getState(),
        brModule.getState());

    // If we are currently cunning a trajectory
    if (trajectoryController.isFinished()) {
      headingController.update(desiredSpeeds, getHeading());
    } else {
      desiredSpeeds = trajectoryController.calculate(getPose());
    }
    setSwerveStates(desiredSpeeds);
    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
  }

  public void drive(
      double xVelocity, double yVelocity, double rotationVelocity, boolean fieldRelative) {

    headingController.stabiliseHeading();

    if (!trajectoryController.isFinished()) trajectoryController.stop();

    if (fieldRelative) {
      desiredSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xVelocity, yVelocity, rotationVelocity, getHeading());
    } else {
      desiredSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
    }
  }

  public void drive(double xVelocity, double yVelocity, Rotation2d rotation) {
    if (!trajectoryController.isFinished()) trajectoryController.stop();
    headingController.lockHeading(rotation);
    // We don't set the rotation speeds as the heading controller will take care of that.
    desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, 0.0, getHeading());
  }

  private void setSwerveStates(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  /**
   * Sets all the swerve module states sequentially.
   *
   * @param states What to set the states to
   */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
    flModule.setState(states[0]);
    frModule.setState(states[1]);
    blModule.setState(states[2]);
    brModule.setState(states[3]);
  }

  /**
   * Get the states of all the current modules.
   *
   * @return A list of SwerveModulesState(s)
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = flModule.getState();
    states[1] = frModule.getState();
    states[2] = blModule.getState();
    states[3] = brModule.getState();
    return states;
  }

  /** Zeros the gyro heading. */
  public void zeroHeading() {
    setHeading(new Rotation2d());
  }

  private void setHeading(Rotation2d heading) {
    gyroAngleAdjustment = heading.rotateBy(getRawHeading().unaryMinus());
  }

  private Rotation2d getRawHeading() {
    // For the Pigeon 1 use getFusedHeading the Pigeon 2 uses getYaw. See the Pigeon 2 user guide.
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns the swerve drive's heading as a Rotation2d.
   *
   * @return A Rotation2d representing the swerve drive's heading
   */
  public Rotation2d getHeading() {
    return gyroAngleAdjustment.rotateBy(getRawHeading());
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get the current pose of the robot.
   *
   * @return A Pose2d that represents the position of the robot
   */
  public Pose2d getPose() {
    return poseEstimator.getPoseMeters();
  }

  public void setPose(Pose2d pose, Rotation2d heading) {
    setHeading(heading);
    poseEstimator.resetPosition(pose, getHeading());
  }

  public void resetSensors() {
    zeroHeading();
    poseEstimator.resetPosition(new Pose2d(), new Rotation2d());
  }

  /**
   * Start following a trajectory.
   *
   * @param trajectory The trajectory to follow.
   */
  public void followTrajectory(PathPlannerTrajectory trajectory) {
    trajectoryController.startTrajectory(trajectory);
  }

  /**
   * Returns if the trajectory controller is finished.
   *
   * @return True if the trajectory is finished. False otherwise.
   * @see TrajectoryController
   */
  public boolean isTrajectoryFinished() {
    return trajectoryController.isFinished();
  }
}
