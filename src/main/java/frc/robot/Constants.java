// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static final int startupCanTimeout = 100; // ms
  public static final int canTimeout = 10; // ms
  public static final double loopTime = 0.02; // s
  public static final int talonPrimaryPid = 0;
  public static final int falconCPR = 2048; // counts per revolution
  public static final double intakeSpeed = 0.40;
  public static final double feederSpeed = 0.50;
  public static final double acceleratorSpeed = 0.20;
  public static final double flywheelSpeed = 0.50;

  public static final class ID {
    public static final int
    ACCELERATOR_MOTOR_ID = 40,
    ACCELERATOR_FOLLOWER_MOTOR_ID = 41,
    FEEDER_MOTOR_ID = 30,
    INTAKE_MOTOR_ID = 20,
    SHOOTER_MOTOR_ID = 50,
    SHOOTER_FOLLOWER_MOTOR_ID = 51,
    HOOD_SERVO_PWM_PORT_ID = 0;
  }
}
