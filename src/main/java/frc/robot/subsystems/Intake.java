package frc.robot.subsystems;

import static frc.robot.Constants.ID.INTAKE_MOTOR_ID;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import io.github.oblarg.oblog.Loggable;

public class Intake extends SubsystemBase {

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

  public double getVelocity() {
    return falconToRPM(intakeMotor.getSelectedSensorVelocity(), INTAKE_GEAR_RATIO);
  }

  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
