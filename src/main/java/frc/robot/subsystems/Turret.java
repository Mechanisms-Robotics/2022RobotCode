package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ID.TURRET_MOTOR_ID;
import static frc.robot.Constants.startupCanTimeout;

public class Turret extends SubsystemBase {

    private static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();
    /* 0 degrees absolute is facing forward, -135 and +135 each direction are bounds */
    private static final int LEFT_BOUND = 0;
    private static final int RIGHT_BOUND = 0; // TODO: get values translated from motor + gear ratio

    static {
        // TODO: Determine how much current the turret draws nominally and
        final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();
        feederCurrentLimit.currentLimit = 10; // Amps
        feederCurrentLimit.triggerThresholdCurrent = 15; // Amps
        feederCurrentLimit.triggerThresholdTime = 0.5; // sec
        feederCurrentLimit.enable = true;
        TURRET_MOTOR_CONFIG.supplyCurrLimit = feederCurrentLimit;
    }

    private final WPI_TalonFX turretMotor = new WPI_TalonFX(TURRET_MOTOR_ID);

    private int currentPos = 0;

    public Turret() {
        turretMotor.configAllSettings(TURRET_MOTOR_CONFIG, startupCanTimeout);
        turretMotor.setInverted(TalonFXInvertType.Clockwise);
        turretMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void recenter() {
        turretMotor.set(ControlMode.Position, 0.0);
    }

    public void stop() {
        turretMotor.set(ControlMode.PercentOutput, 0.0);
    }

}
