package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;
import static frc.robot.util.Units.RPMToFalcon;
import static frc.robot.util.Units.falconToRPM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import io.github.oblarg.oblog.Loggable;

public class Intake extends SubsystemBase {

    private static final int INTAKE_MOTOR_ID = 20;
    private static final int INTAKE_SOLENOID_FORWARD_ID = 5;
    private static final int INTAKE_SOLENOID_REVERSE_ID = 2;
    private static final DoubleSolenoid.Value INTAKE_DEPLOYED = DoubleSolenoid.Value.kForward;
    private static final DoubleSolenoid.Value INTAKE_RETRACTED = DoubleSolenoid.Value.kReverse;
    private static final double INTAKE_GEAR_RATIO = 1.0;

    private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
    private static final int VELOCITY_PID_SLOT = 0;

    static {
        // TODO: Determine how much current the intake draws nominally and
        final var intakeCurrentLimit = new SupplyCurrentLimitConfiguration();
        intakeCurrentLimit.currentLimit = 10; // Amps
        intakeCurrentLimit.triggerThresholdCurrent = 15; // Amps
        intakeCurrentLimit.triggerThresholdTime = 0.5; // sec
        intakeCurrentLimit.enable = true;
        INTAKE_MOTOR_CONFIG.supplyCurrLimit = intakeCurrentLimit;

        // TODO: Tune
        final var velocityLoopConfig = new SlotConfiguration();
        velocityLoopConfig.kP = 0.0;
        velocityLoopConfig.kI = 0.0;
        velocityLoopConfig.kD = 0.0;
        INTAKE_MOTOR_CONFIG.slot0 = velocityLoopConfig;
    }

    private final WPI_TalonFX intakeMotor = new WPI_TalonFX(INTAKE_MOTOR_ID);
    private final DoubleSolenoid intakeSolenoid =
            new DoubleSolenoid(PneumaticsModuleType.REVPH, INTAKE_SOLENOID_FORWARD_ID, INTAKE_SOLENOID_REVERSE_ID);

    public Intake() {
        intakeMotor.configAllSettings(INTAKE_MOTOR_CONFIG, startupCanTimeout);
        intakeMotor.setInverted(TalonFXInvertType.Clockwise);
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.selectProfileSlot(VELOCITY_PID_SLOT, 0);
    }

    public void setOpenLoop(double percentOutput) {
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setVelocity(double rpm) {
        intakeMotor.set(ControlMode.Velocity, RPMToFalcon(rpm, INTAKE_GEAR_RATIO));
    }

    public void deploy() {
        intakeSolenoid.set(INTAKE_DEPLOYED);
    }

    public void retract() {
        stop();
        intakeSolenoid.set(INTAKE_RETRACTED);
    }

    public boolean isDeployed() {
        return intakeSolenoid.get() == INTAKE_DEPLOYED;
    }

    public double getVelocity() {
        return falconToRPM(intakeMotor.getSelectedSensorVelocity(), INTAKE_GEAR_RATIO);
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }
}