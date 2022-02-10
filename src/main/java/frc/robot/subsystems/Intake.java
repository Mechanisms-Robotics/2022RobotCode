package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static final double INTAKE_SPEED = 0.50;
  private static final double OUTTAKE_SPEED = -0.25;

  private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();

  static {
    final var intakeCurrentLimit = new SupplyCurrentLimitConfiguration();
    intakeCurrentLimit.currentLimit = 10; // Amps
    intakeCurrentLimit.triggerThresholdCurrent = 15; // Amps
    intakeCurrentLimit.triggerThresholdTime = 0.5; // sec
    intakeCurrentLimit.enable = true;
    INTAKE_MOTOR_CONFIG.supplyCurrLimit = intakeCurrentLimit;
  }

  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(20);

  public Intake() {
    intakeMotor.configAllSettings(INTAKE_MOTOR_CONFIG, startupCanTimeout);
    intakeMotor.setInverted(TalonFXInvertType.Clockwise);
    intakeMotor.setNeutralMode(NeutralMode.Coast);
  }

  private void setOpenLoop(double percentOutput) {
    intakeMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void intake() {
    setOpenLoop(INTAKE_SPEED);
  }

  public void outtake() {
    setOpenLoop(OUTTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
