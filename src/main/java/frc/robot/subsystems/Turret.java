package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Units;

import static frc.robot.Constants.ID.TURRET_MOTOR_ID;
import static frc.robot.Constants.startupCanTimeout;

public class Turret extends SubsystemBase {

    private static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();

    private static final Rotation2d BOUND = Rotation2d.fromDegrees(135.0);
    private static final double TURRET_GEAR_RATIO = 41.66;
    private static final double TURRET_ROTATION_LIMIT_SWITCH = 135.0;

    static {
        // TODO: Determine how much current the turret draws nominally and
        final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();
        feederCurrentLimit.currentLimit = 10; // Amps
        feederCurrentLimit.triggerThresholdCurrent = 15; // Amps
        feederCurrentLimit.triggerThresholdTime = 0.5; // sec
        feederCurrentLimit.enable = true;
        TURRET_MOTOR_CONFIG.supplyCurrLimit = feederCurrentLimit;

        TURRET_MOTOR_CONFIG.forwardSoftLimitThreshold = Units.degreesToFalcon(TURRET_ROTATION_LIMIT_SWITCH, TURRET_GEAR_RATIO);
        TURRET_MOTOR_CONFIG.reverseSoftLimitThreshold = Units.degreesToFalcon(TURRET_ROTATION_LIMIT_SWITCH, TURRET_GEAR_RATIO);
        TURRET_MOTOR_CONFIG.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    }

    private final WPI_TalonFX turretMotor = new WPI_TalonFX(TURRET_MOTOR_ID);

    private double currentAzimuth = 0.0;

    public Turret() {
        turretMotor.configAllSettings(TURRET_MOTOR_CONFIG, startupCanTimeout);
        turretMotor.setInverted(TalonFXInvertType.Clockwise);
        turretMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets position of the turret from 135 to -135 degrees.
     * @param azimuth
     */
    public void setPosition(double azimuth) {
        turretMotor.set(ControlMode.Position, Units.degreesToFalcon(MathUtil.clamp(this.currentAzimuth = azimuth, -BOUND.getDegrees(), BOUND.getDegrees()), TURRET_GEAR_RATIO));
    }

    public double getPosition() {
        return this.currentAzimuth;
    }

    /**
     * Offsets the position of the turret by specified degrees.
     * Resulting azimuth must be within turret bounds.
     * @param degrees
     */
    public void offset(double degrees) {
        setPosition(getPosition() + degrees);
    }

    /**
     * Sets the turret rotation facing forward at 0 degrees.
     */
    public void recenter() {
        setPosition(0.0);
    }

    public void stop() {
        turretMotor.set(ControlMode.PercentOutput, 0.0);
    }

}
