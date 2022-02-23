package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Units;

/** This class contains all code responsible for the behaviour of the Turret subsystem. */
public class Turret extends SubsystemBase {
  // Motor configuration for the turret motor
  private static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();

  private static final double TURRET_GEAR_RATIO = 50.0; // 50:1
  private static final double TURRET_FORWARD_LIMIT = Math.toRadians(0.0); // 0 degrees
  private static final double TURRET_REVERSE_LIMIT = Math.toRadians(-270.0); // -270 degrees
  private static final double TURRET_ALLOWABLE_ERROR = Math.toRadians(1.0); // 1 degree

  private boolean zeroed = false; // Has the turret been zeroed

  private double currentAngle = 0.0 * Math.PI; // rads
  private double desiredAngle = 0.0 * Math.PI; // rads
  private boolean aimed = false; // Is the turret aimed at the desired angle

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

    // Turret motor PID configuration
    final var turretPID = new SlotConfiguration();
    turretPID.kP = 0.5;
    turretPID.allowableClosedloopError =
        Units.radsToFalcon(TURRET_ALLOWABLE_ERROR, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIG.slot0 = turretPID;

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

    turretMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  /**
   * Aims the turret toward a target
   *
   * @param degrees The angle between the current turret angle and the desired turret angle
   */
  public void aim(double degrees) {
    this.desiredAngle =
        MathUtil.clamp(
            this.currentAngle + Math.toRadians(degrees),
            TURRET_REVERSE_LIMIT,
            TURRET_FORWARD_LIMIT);

    setPosition(this.desiredAngle);
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

    // Update the currentAngle
    this.currentAngle =
        Units.falconToRads(turretMotor.getSelectedSensorPosition(), TURRET_GEAR_RATIO);

    // Check if we're at the desired angle and update aimed
    this.aimed =
        MathUtil.applyDeadband(this.desiredAngle - this.currentAngle, TURRET_ALLOWABLE_ERROR)
            == 0.0;

    // TODO: Fix angle bug and remove this debug output
    System.out.println("Turret Setpoint" + Units.radsToFalcon(rads, TURRET_GEAR_RATIO));
    SmartDashboard.putNumber("Turret Setpoint", Units.radsToFalcon(rads, TURRET_GEAR_RATIO));
  }

  /**
   * Checks if the turret is aimed
   *
   * @return Is the turret aimed at the desired angle
   */
  public boolean isAimed() {
    return this.aimed;
  }

  /** Zeros the turret encoder, this is called in autonomousInit and testInit */
  public void zero() {
    turretMotor.setSelectedSensorPosition(0.0);
    zeroed = true;
  }

  /** Stops the turret */
  public void stop() {
    turretMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
