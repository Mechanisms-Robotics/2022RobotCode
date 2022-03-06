package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.Units;

/** This class contains all the code responsible for the behaviour of the Shooter subsystem */
public class Shooter extends SubsystemBase {

  // Shooter motor configuration
  private static final TalonFXConfiguration SHOOTER_MOTOR_CONFIG = new TalonFXConfiguration();

  // Shooter gear ratio
  private static final double GEAR_RATIO = 1.5; // 1.5:1 reduction

  private static final int SHOOTER_SPINUP_DEADBAND = Units.RPMToFalcon(50, GEAR_RATIO);

  // Range interpolating tree map
  private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> RANGE_TO_RPM =
      new InterpolatingTreeMap<>();

  // Default shooter speed
  private static final double DEFAULT_SHOOTER_VEL = 1500.0; // RPM
  private static final double BACKUP_SPEED = -0.25; // percent

  // Shooter feedforward
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          0.52017, // ks
          0.000084452, // kv
          0.0000089707 // ka
          );

  // Configure shooter current limit, PID, and interpolating tree map
  static {
    // Configure shooter current limit
    final var shooterCurrentLimit = new SupplyCurrentLimitConfiguration();
    shooterCurrentLimit.currentLimit = 40; // Amps
    shooterCurrentLimit.triggerThresholdCurrent = 45; // Amps
    shooterCurrentLimit.triggerThresholdTime = 0.5; // sec
    shooterCurrentLimit.enable = true;
    SHOOTER_MOTOR_CONFIG.supplyCurrLimit = shooterCurrentLimit;

    // Configure shooter PID
    final var shooterPID = new SlotConfiguration();
    shooterPID.kP = 0.15; // 0.075
    shooterPID.kF = 0.055;
    SHOOTER_MOTOR_CONFIG.slot0 = shooterPID;

    // Configure shooter range interpolating tree map (meters, RPM)
    RANGE_TO_RPM.put(new InterpolatingDouble(0.0), new InterpolatingDouble(1500.0));
    RANGE_TO_RPM.put(new InterpolatingDouble(0.6), new InterpolatingDouble(1500.0));
    RANGE_TO_RPM.put(new InterpolatingDouble(1.1), new InterpolatingDouble(1750.0));
    // RANGE_TO_RPM.put(new InterpolatingDouble(1.72), new InterpolatingDouble(2000.0));
    // RANGE_TO_RPM.put(new InterpolatingDouble(2.82), new InterpolatingDouble(2000.0));
    RANGE_TO_RPM.put(new InterpolatingDouble(20.0), new InterpolatingDouble(3000.0));

    // Configure shooter velocity measurement
    SHOOTER_MOTOR_CONFIG.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
    SHOOTER_MOTOR_CONFIG.velocityMeasurementWindow = 4;
  }

  // Shooter master and follower motor
  private final WPI_TalonFX shooterMotor = new WPI_TalonFX(50);
  private final WPI_TalonFX shooterFollowerMotor = new WPI_TalonFX(51);

  /** Constructs a Shooter */
  public Shooter() {
    // Configure shooter master motor
    shooterMotor.configAllSettings(SHOOTER_MOTOR_CONFIG, startupCanTimeout);
    shooterMotor.setInverted(TalonFXInvertType.Clockwise);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

    // Configure shooter follower motor
    shooterFollowerMotor.configAllSettings(SHOOTER_MOTOR_CONFIG, startupCanTimeout);
    shooterFollowerMotor.follow(shooterMotor);
    shooterFollowerMotor.setInverted(InvertType.OpposeMaster);
    shooterFollowerMotor.setNeutralMode(NeutralMode.Coast);

    // CAN bus utilization optimisation
    shooterFollowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    shooterFollowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  /**
   * Sets the shooter motors to run at a desired percentage
   *
   * @param percentOutput The percentage to run the shooter motors at
   */
  private void setOpenLoop(double percentOutput) {
    shooterMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Runs the shooter at a calculated RPM given a range to the target
   *
   * @param range Range to target
   */
  public void shoot(double range) {
    // Interpolate RPM given range
    final double velocity = RANGE_TO_RPM.getInterpolated(new InterpolatingDouble(range)).value;

    // Run shooter motor at velocity
    shooterMotor.set(ControlMode.Velocity, Units.RPMToFalcon(velocity, GEAR_RATIO));
  }

  /** Runs the shooter at the default RPM */
  public void shoot() {
    shooterMotor.set(ControlMode.Velocity, Units.RPMToFalcon(DEFAULT_SHOOTER_VEL, GEAR_RATIO));
  }

  /** Backs up the shooter in case of a jam */
  public void backup() {
    shooterMotor.set(ControlMode.PercentOutput, BACKUP_SPEED);
  }

  /** Stops the shooter */
  public void stop() {
    shooterMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean atSpeed() {
    if (shooterMotor.getControlMode().equals(ControlMode.Velocity)) {
      return Math.abs(shooterMotor.getClosedLoopError()) <= SHOOTER_SPINUP_DEADBAND;
    }
    return true;
  }
}
