package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.Units;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasVoltageCompensation()) {
                motorConfiguration.voltageCompSaturation = nominalVoltage;
            }

            if (hasCurrentLimit()) {
                motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
                motorConfiguration.supplyCurrLimit.enable = true;
            }

            TalonFX motor = new TalonFX(driveConfiguration);
            CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");

            if (hasVoltageCompensation()) {
                // Enable voltage compensation
                motor.enableVoltageCompensation(true);
            }

            motor.setNeutralMode(NeutralMode.Coast);

            motor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
            motor.setSensorPhase(true);

            // TODO: Rewrite SDS base code to make this cleaner
            motor.config_kP(0,  0.0007298);
            motor.selectProfileSlot(0, 0);

            // Reduce CAN status frame rates
            CtreUtils.checkCtreError(
                    motor.setStatusFramePeriod(
                            StatusFrameEnhanced.Status_1_General,
                            STATUS_FRAME_GENERAL_PERIOD_MS,
                            CAN_TIMEOUT_MS
                    ),
                    "Failed to configure Falcon status frame period"
            );

            return new ControllerImplementation(motor, sensorVelocityCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        // TODO: Rewrite SDS Base Code to Make this cleaner
        private final SimpleMotorFeedforward wheelFeedforward = new SimpleMotorFeedforward(
            0.319185544,
            2.2544,
            0.063528
        );

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
        }

        @Override
        public void setSpeed(double speedMPS) {
            motor.set(TalonFXControlMode.Velocity,
                speedMPS,
                DemandType.ArbitraryFeedForward,
                wheelFeedforward.calculate(speedMPS) / 12.0);
        }

        @Override
        public double getVelocity() {
            return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
        }
    }
}