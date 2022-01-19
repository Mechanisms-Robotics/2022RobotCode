// Code based on Team 3847
// https://github.com/Spectrum3847/GammaRay-2021/blob/main/src/main/java/frc/robot/subsystems/SwerveModule.java

package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.util.Units.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.CTREModuleState;

/** A hardware wrapper class for a swerve module that uses Falcon500s. */
public class SwerveModule /*implements Loggable*/ {

    private static final double WHEEL_GEAR_RATIO = 6.86; // : 1
    private static final double WHEEL_DIAMETER = 0.1016; // meters
    private static final double STEER_GEAR_RATIO = 12.8; // : 1
    private static final int VELOCITY_PID_SLOT = 0;
    private static final int MOTION_MAGIC_PID_SLOT = 1;

    private static final TalonFXConfiguration WHEEL_MOTOR_CONFIG = new TalonFXConfiguration();
    private static final TalonFXConfiguration STEERING_MOTOR_CONFIG = new TalonFXConfiguration();
    private static final CANCoderConfiguration CONFIGURATION = new CANCoderConfiguration();

    private final SimpleMotorFeedforward wheelFeedforward = new SimpleMotorFeedforward(
        0.0,
        0.0,
        0.0
    );

    static {
        // Wheel Motor Current Limiting
        var wheelMotorCurrentLimit = new SupplyCurrentLimitConfiguration();
        wheelMotorCurrentLimit.enable = true;
        wheelMotorCurrentLimit.currentLimit = 30; // Amps
        wheelMotorCurrentLimit.triggerThresholdCurrent = 35; // Amps
        wheelMotorCurrentLimit.triggerThresholdTime = 0.5; // Seconds
        WHEEL_MOTOR_CONFIG.supplyCurrLimit = wheelMotorCurrentLimit;

        // Configure wheel motor current limiting
        WHEEL_MOTOR_CONFIG.voltageCompSaturation = 12.0; // Volts

        var wheelPID = new SlotConfiguration();
        wheelPID.kP = 0.08; // 0.012

        // TODO: Remove once velocity feedforward is calculated
        wheelPID.kF = 0.05;

        WHEEL_MOTOR_CONFIG.slot0 = wheelPID;

        WHEEL_MOTOR_CONFIG.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_50Ms;
        WHEEL_MOTOR_CONFIG.velocityMeasurementWindow = 8;

        // Configure steering motor current limiting
        var steeringMotorCurrentLimit = new SupplyCurrentLimitConfiguration();
        steeringMotorCurrentLimit.enable = true;
        steeringMotorCurrentLimit.currentLimit = 15; // Amps
        steeringMotorCurrentLimit.triggerThresholdCurrent = 25; // Amps
        steeringMotorCurrentLimit.triggerThresholdTime = 0.5; // Seconds
        STEERING_MOTOR_CONFIG.supplyCurrLimit = steeringMotorCurrentLimit;

        STEERING_MOTOR_CONFIG.voltageCompSaturation = 8.0; // Volts

        var steeringPID = new SlotConfiguration();
        steeringPID.kP = 0.6; // 0.3
        steeringPID.kI = 0.0; // 0.00012
        steeringPID.kD = 12.0; // 3.0
        steeringPID.kF = 0.0; // 0.0008
        steeringPID.allowableClosedloopError = 10; // ticks
        STEERING_MOTOR_CONFIG.slot1 = steeringPID;

        STEERING_MOTOR_CONFIG.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_50Ms;
        STEERING_MOTOR_CONFIG.velocityMeasurementWindow = 64;

        CONFIGURATION.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        CONFIGURATION.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        CONFIGURATION.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    private final WPI_TalonFX wheelMotor;
    private final WPI_TalonFX steerMotor;
    private final CANCoder steeringEncoder;
    private final String moduleName;

    private double lastAngle; // For debugging
    private final double angleOffset; // This is only used to calibrate the swerve modules

    /**
     * Constructs a swerve module.
     *
     * @param name The unique name of this swerve module
     * @param wheelMotorId The CAN ID of the Spark Max used to control the wheel
     * @param steeringMotorId The CAN ID of the Spark Max used to control the steering
     * @param angleEncoderId The CAN ID of the CANCoder used to determine the angle of the module
     */
    public SwerveModule(
            String name, int wheelMotorId, int steeringMotorId, int angleEncoderId, double angleOffset) {

        moduleName = name;
        this.angleOffset = angleOffset;

        // Setup wheel motor
        wheelMotor = new WPI_TalonFX(wheelMotorId);
        wheelMotor.configAllSettings(WHEEL_MOTOR_CONFIG, startupCanTimeout);
        wheelMotor.setInverted(TalonFXInvertType.CounterClockwise);
        wheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, startupCanTimeout);
        wheelMotor.selectProfileSlot(VELOCITY_PID_SLOT, 0);
        wheelMotor.setNeutralMode(NeutralMode.Brake);

        // Setup steering encoder
        steeringEncoder = new CANCoder(angleEncoderId);
        steeringEncoder.configAllSettings(CONFIGURATION);

        // Setup steering motor
        steerMotor = new WPI_TalonFX(steeringMotorId);
        steerMotor.configAllSettings(STEERING_MOTOR_CONFIG, startupCanTimeout);
        steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
        steerMotor.setNeutralMode(NeutralMode.Coast);
        steerMotor.selectProfileSlot(MOTION_MAGIC_PID_SLOT, 0);

        resetToAbsolute();
    }

    /**
     * Get the steering angle based on the absolute encoder
     *
     * @return Steering angle in degrees.
     */
//    @Log.ToString
    public Rotation2d getSteeringAngle() {
        return Rotation2d.fromDegrees(steeringEncoder.getAbsolutePosition() - angleOffset);
    }

    /**
     * Get the steering angle based on the internal falcon encoder.
     *
     * @return Steering angle in degrees.
     */
//    @Log
    public double getSteeringAngleMotor() {
        return falconToDegrees(steerMotor.getSelectedSensorPosition(), STEER_GEAR_RATIO);
    }

    /**
     * Get the wheel velocity in meters per second.
     *
     * @return Swerve wheel velocity in meters per second.
     */
//    @Log
    public double getWheelVelocity() {
        return falconToMPS(
                wheelMotor.getSelectedSensorVelocity(), Math.PI * WHEEL_DIAMETER, WHEEL_GEAR_RATIO);
    }

    /**
     * Get the current state of the module.
     *
     * @return A SwerveModuleState representing the current speed and rotation of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getWheelVelocity(), Rotation2d.fromDegrees(getSteeringAngleMotor()));
    }

    /**
     * Set the current state of the swerve module.
     *
     * @param state The SwerveModuleState to set the module to
     */
    public void setState(SwerveModuleState state) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // CTRE is not
        state = CTREModuleState.optimize(state, getState().angle);
        setSpeed(state.speedMetersPerSecond);

        // The speed of the wheel is really low (less than 0.01% Max speed) don't
        // steer. This prevents jittering
        final double angle =
                (Math.abs(state.speedMetersPerSecond) <= (Swerve.maxVelocity * 0.01))
                        ? lastAngle
                        : state.angle.getDegrees();
        steerMotor.set(ControlMode.Position, degreesToFalcon(angle, STEER_GEAR_RATIO));
        lastAngle = angle;
    }

    /**
     * Set the speed of the wheel.
     * 
     * @param wheelVelocityMPS Speed of the wheel in meters per second
     */
    public void setSpeed(double wheelVelocityMPS) {
        final double velocity =
                MPSToFalcon(wheelVelocityMPS, Math.PI * WHEEL_DIAMETER, WHEEL_GEAR_RATIO);
        wheelMotor.set(ControlMode.Velocity, velocity,
            DemandType.ArbitraryFeedForward, wheelFeedforward.calculate(wheelVelocityMPS));
    }

    /** Resets the steering motor's internal encoder to the value of the absolute encoder. * */
    private void resetToAbsolute() {
        double absolutePosition = degreesToFalcon(getSteeringAngle().getDegrees(), STEER_GEAR_RATIO);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    /** Stop the wheel and steering robot */
    public void stop() {
        steerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        wheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }
}