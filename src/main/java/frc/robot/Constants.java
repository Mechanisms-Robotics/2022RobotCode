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

    // TODO: Change to velocity once PIDs are tuned.
    // Currently open loop precentage
    public static final double shooterShootSpeed = 5200;
    public static final double acceleratorShootSpeed = 0.80;
    public static final double spindexerShootSpeed = 0.80;
    public static final double spindexerIntakeSpeed = 0.15;
    public static final double spindexerPrepSpeed = 0.15;
    public static final double intakeSpeed = 0.70;

    // Auto Constants
    public static final double linearGain = 1.25;
    public static final double headingGain = 2.0;
    public static final double headingMaxVelocity = 2 * Math.PI;
    public static final double headingMaxAccel = 2 * Math.PI;

}
