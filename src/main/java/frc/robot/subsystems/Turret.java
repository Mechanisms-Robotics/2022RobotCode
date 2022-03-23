package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Units;

/** This class contains all code responsible for the behaviour of the Turret subsystem. */
public class Turret extends SubsystemBase {
  // Motor configuration for the turret motor
  private static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();

  private static final double TURRET_GEAR_RATIO = 50.0; // 50:1
  public static final double TURRET_FORWARD_LIMIT = Math.toRadians(0.0); // 0 degrees
  public static final double TURRET_REVERSE_LIMIT = Math.toRadians(-270.0); // -270 degrees
  private static final double TURRET_ALLOWABLE_ERROR = Math.toRadians(0.5); // 0.5 degrees
  private static final double TURRET_AIM_ERROR = Math.toRadians(3.0); // 3 degrees
  private static final double TURRET_AIM_MOVEMENT_SCALAR = 0.125;

  private static final Transform2d ROBOT_TO_TURRET =
      new Transform2d(
          new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
          new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(-90.0)));
  private static final Transform2d TURRET_TO_ROBOT = ROBOT_TO_TURRET.inverse();

  private boolean snapAroundEnabled = true;
  private boolean zeroed = false; // Has the turret been zeroed

  private double desiredAngle = 0.0; // rads

  static {
    // Current limit configuration for the turret motor
    final var turretCurrentLimit = new SupplyCurrentLimitConfiguration();
    turretCurrentLimit.currentLimit = 10; // Amps
    turretCurrentLimit.triggerThresholdCurrent = 15; // Amps
    turretCurrentLimit.triggerThresholdTime = 0.5; // sec
    turretCurrentLimit.enable = true;
    TURRET_MOTOR_CONFIG.supplyCurrLimit = turretCurrentLimit;

    // Turret motor soft limits
    TURRET_MOTOR_CONFIG.forwardSoftLimitThreshold =
        Units.radsToFalcon(TURRET_FORWARD_LIMIT, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIG.reverseSoftLimitThreshold =
        Units.radsToFalcon(TURRET_REVERSE_LIMIT, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIG.forwardSoftLimitEnable = true;
    TURRET_MOTOR_CONFIG.reverseSoftLimitEnable = true;

    // Turret motion magic PID configuration
    final var turretMMPID = new SlotConfiguration();
    turretMMPID.kP = 0.5;
    turretMMPID.allowableClosedloopError =
        Units.radsToFalcon(TURRET_ALLOWABLE_ERROR, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIG.slot0 = turretMMPID;

    // Turret position PID configuration
    final var turretPositionPID = new SlotConfiguration();
    turretPositionPID.kP = 0.1;
    turretPositionPID.kD = 1.0;
    turretPositionPID.allowableClosedloopError =
        Units.radsToFalcon(TURRET_ALLOWABLE_ERROR, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIG.slot1 = turretPositionPID;

    // Turret motor Motion Magic configuration
    TURRET_MOTOR_CONFIG.motionAcceleration =
        (Units.radsToFalcon(2 * Math.PI, TURRET_GEAR_RATIO) / 10.0) * 5; // 10π rads/sec
    TURRET_MOTOR_CONFIG.motionCruiseVelocity =
        (Units.radsToFalcon(2 * Math.PI, TURRET_GEAR_RATIO) / 10.0) * 2.5; // 5π rads/sec
    TURRET_MOTOR_CONFIG.motionCurveStrength = 1; // 1-8, 8 being maximum smoothing

    // Turret motor velocity measurement configuration
    TURRET_MOTOR_CONFIG.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
    TURRET_MOTOR_CONFIG.velocityMeasurementWindow = 4;
  }

  // Turret motor
  private final WPI_TalonFX turretMotor = new WPI_TalonFX(60);

  /** Constructs the turret object and configures the turret motor */
  public Turret() {
    turretMotor.configAllSettings(TURRET_MOTOR_CONFIG, startupCanTimeout);
    turretMotor.setInverted(TalonFXInvertType.Clockwise);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.selectProfileSlot(1, 0);

    turretMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  /**
   * Aims the turret toward a target
   *
   * @param degrees The angle between the current turret angle and the desired turret angle
   */
  public void aim(double degrees, ChassisSpeeds velocity, double range) {
    this.desiredAngle =
        MathUtil.clamp(
            Units.falconToRads(turretMotor.getSelectedSensorPosition(), TURRET_GEAR_RATIO)
                + Math.toRadians(degrees),
            TURRET_REVERSE_LIMIT,
            TURRET_FORWARD_LIMIT);

    setPosition(this.desiredAngle);

    SmartDashboard.putNumber("Current Angle", getAngle());
    SmartDashboard.putNumber("Desired Angle", desiredAngle);

    if (isAimed()) {
      return;
    }

    setPosition(desiredAngle);
  }

  /** Snaps the turret to the angle (radians) */
  public void snapTo(double angle) {
    desiredAngle = MathUtil.clamp(angle, TURRET_REVERSE_LIMIT, TURRET_FORWARD_LIMIT);

    if (isAimed()) {
      return;
    }

    setPosition(desiredAngle);
  }

  /** Snaps to limit farthest from current angle */
  public void snapAround() {
    if (!snapAroundEnabled) {
      return;
    }

    if (Math.abs(TURRET_FORWARD_LIMIT - getAngle())
        >= Math.abs(TURRET_REVERSE_LIMIT - getAngle())) {
      snapTo(TURRET_FORWARD_LIMIT);
    } else {
      snapTo(TURRET_REVERSE_LIMIT);
    }
  }

  /** Enables snap around functionality */
  public void enableSnapAround() {
    this.snapAroundEnabled = true;
  }

  /** Disables snap around functionality */
  public void disableSnapAround() {
    this.snapAroundEnabled = false;
  }

  /** Sets the turret to the zero position */
  public void goToZero() {
    setPosition(0.0);
  }

  /**
   * PIDs the turret motor to an angle
   *
   * @param rads Target angle
   */
  private void setPosition(double rads) {
    // If the turret has not been zeroed return
    if (!zeroed) {
      return;
    }

    // PID the turret motor to the desired position
    turretMotor.set(ControlMode.MotionMagic, Units.radsToFalcon(rads, TURRET_GEAR_RATIO));
  }

  /**
   * Checks if the turret is aimed
   *
   * @return Is the turret aimed at the desired angle
   */
  public boolean isAimed() {
    if (turretMotor.getControlMode().equals(ControlMode.Position)
        || turretMotor.getControlMode().equals(ControlMode.MotionMagic)) {
      SmartDashboard.putBoolean(
          "Is Aimed", Math.abs(getAngle() - desiredAngle) <= TURRET_AIM_ERROR);
      return Math.abs(getAngle() - desiredAngle) <= TURRET_AIM_ERROR;
    }
    SmartDashboard.putBoolean("Is Aimed", false);
    return false;
  }

  /**
   * Adjusts an angle based on the velocity of the robot and a range to the target
   *
   * @param angle The angle to adjust
   * @param velocity The velocity of the robot
   * @param range The range to the target
   * @return The adjusted angle
   */
  private double adjustAngleForVelocity(double angle, ChassisSpeeds velocity, double range) {
    // Split up the velocity into translational and rotational velocities
    Translation2d translationalVelocity =
        new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
    double rotationalVelocity = velocity.omegaRadiansPerSecond;

    // Rotate the translational velocity vector so the x axis points at the target
    translationalVelocity.rotateBy(TURRET_TO_ROBOT.getRotation().rotateBy(new Rotation2d(angle)));

    // Calculate the angle offset, add it to the angle and return that
    double offset = (TURRET_AIM_MOVEMENT_SCALAR * translationalVelocity.getY()) / range;
    return angle + offset;
  }

  public double getAngle() {
    return Units.falconToRads(turretMotor.getSelectedSensorPosition(), TURRET_GEAR_RATIO);
  }

  /** Zeros the turret encoder, this is called in autonomousInit and testInit */
  public void zero() {
    if (zeroed) {
      return;
    }
    turretMotor.setSelectedSensorPosition(0.0);
    zeroed = true;
  }

  /** Stops the turret */
  public void stop() {
    turretMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
