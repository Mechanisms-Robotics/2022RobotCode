package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorTestSubsystem extends SubsystemBase {

  private final TalonFX motor;

  public SingleMotorTestSubsystem(int motorID) {
    motor = new TalonFX(motorID);
  }

  public void set(double value) {
    System.out.println("***********MOTOR***********");
    motor.set(ControlMode.PercentOutput, value);
  }

  public void setInverted() {
    motor.setInverted(!motor.getInverted());
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
