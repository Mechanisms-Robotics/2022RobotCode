package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  private static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();

  private static final double TURRET_GEAR_RATIO = 41.66;
  private static final double TURRET_ROTATION_LIMIT = 180.0;

  private boolean zerod = false;

  static {
    final var shooterCurrentLimit = new SupplyCurrentLimitConfiguration();
    shooterCurrentLimit.currentLimit = 10; // Amps
    shooterCurrentLimit.triggerThresholdCurrent = 15; // Amps
    shooterCurrentLimit.triggerThresholdTime = 0.5; // sec
    shooterCurrentLimit.enable = true;
    TURRET_MOTOR_CONFIG.supplyCurrLimit = shooterCurrentLimit;

    TURRET_MOTOR_CONFIG.forwardSoftLimitThreshold = 1;
    TURRET_MOTOR_CONFIG.reverseSoftLimitThreshold = -1;
    TURRET_MOTOR_CONFIG.forwardSoftLimitEnable = true;
    TURRET_MOTOR_CONFIG.reverseSoftLimitEnable = true;
  }

  private final WPI_TalonFX turretMotor = new WPI_TalonFX(60);

  public Turret() {
    turretMotor.configAllSettings(TURRET_MOTOR_CONFIG, startupCanTimeout);
    turretMotor.setInverted(TalonFXInvertType.Clockwise);
    turretMotor.setNeutralMode(
        NeutralMode.Brake); // Should we use cost mode once the turret can move?

    turretMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  public void aim(Rotation2d aimingPos) {
    // Logic for making sure the turret is aimed.
  }

  private void setPosition(double rads) {
    // Use Motion Magic with a big S-Curve strength
  }

  public boolean isAimed() {
    // Is the turret aimed?
    return false;
  }

  public void stop() {
    turretMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
