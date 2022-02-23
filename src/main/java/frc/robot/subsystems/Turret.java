package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Units;

public class Turret extends SubsystemBase {

  private static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();

  private static final double TURRET_GEAR_RATIO = 50;
  private static final double TURRET_FORWARD_LIMIT = Math.toRadians(0.0);
  private static final double TURRET_REVERSE_LIMIT = Math.toRadians(-270.0);

  private boolean zerod = false;

  static {
    final var turretCurrentLimit = new SupplyCurrentLimitConfiguration();
    turretCurrentLimit.currentLimit = 10; // Amps
    turretCurrentLimit.triggerThresholdCurrent = 15; // Amps
    turretCurrentLimit.triggerThresholdTime = 0.5; // sec
    turretCurrentLimit.enable = true;
    TURRET_MOTOR_CONFIG.supplyCurrLimit = turretCurrentLimit;

    TURRET_MOTOR_CONFIG.forwardSoftLimitThreshold = Units.radsToFalcon(TURRET_FORWARD_LIMIT, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIG.reverseSoftLimitThreshold = Units.radsToFalcon(TURRET_REVERSE_LIMIT, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIG.forwardSoftLimitEnable = true;
    TURRET_MOTOR_CONFIG.reverseSoftLimitEnable = true;

    final var turretPID = new SlotConfiguration();
    turretPID.kP = 0.5;
    TURRET_MOTOR_CONFIG.slot0 = turretPID;

    TURRET_MOTOR_CONFIG.motionAcceleration = (Units.radsToFalcon(2 * Math.PI, TURRET_GEAR_RATIO) / 10.0) * 5; // units/100ms
    TURRET_MOTOR_CONFIG.motionCruiseVelocity = (Units.radsToFalcon(2 * Math.PI, TURRET_GEAR_RATIO) / 10.0) * 2.5; // units/100ms
    TURRET_MOTOR_CONFIG.motionCurveStrength = 1; // 1-8, 8 being maximum smoothing

    TURRET_MOTOR_CONFIG.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
    TURRET_MOTOR_CONFIG.velocityMeasurementWindow = 4;
  }

  private final WPI_TalonFX turretMotor = new WPI_TalonFX(60);

  public Turret() {
    turretMotor.configAllSettings(TURRET_MOTOR_CONFIG, startupCanTimeout);
    turretMotor.setInverted(TalonFXInvertType.Clockwise);
    turretMotor.setNeutralMode(
        NeutralMode.Brake);

    turretMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  public void aim(double degrees) {
    // Logic for making sure the turret is aimed.
    setPosition(Math.toRadians(MathUtil.clamp(degrees, Math.toDegrees(TURRET_REVERSE_LIMIT), Math.toDegrees(TURRET_FORWARD_LIMIT))));
  }

  private void setPosition(double rads) {
    // Use Motion Magic with a big S-Curve strength
    if (!zerod) {
      return;
    }
    System.out.println("Turret Setpoint" + Units.radsToFalcon(rads, TURRET_GEAR_RATIO));
    SmartDashboard.putNumber("Turret Setpoint", Units.radsToFalcon(rads, TURRET_GEAR_RATIO));
    turretMotor.set(ControlMode.MotionMagic, Units.radsToFalcon(rads, TURRET_GEAR_RATIO));
  }

  public boolean isAimed() {
    // Is the turret aimed?
    return false;
  }

  public void zero() {
    turretMotor.setSelectedSensorPosition(0.0);
    zerod = true;
  }

  public void stop() {
    turretMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
