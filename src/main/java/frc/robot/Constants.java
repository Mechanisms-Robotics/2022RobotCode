// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public final class Constants {

  public static final int startupCanTimeout = 255; // ms
  public static final int canTimeout = 10; // ms
  public static final double loopTime = 0.02; // s
  public static final int talonPrimaryPid = 0;
  public static final int falconCPR = 2048; // counts per revolution
  public static final Transform2d fieldRobot =
      new Transform2d(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
          new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));
}
