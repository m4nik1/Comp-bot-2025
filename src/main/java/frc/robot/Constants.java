// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Swerve system constants
  public static final double wheelBase = Units.inchesToMeters(21);
  public static final double trackWidth = Units.inchesToMeters(21);
  public static final double driveBaseRadius = Math.hypot(wheelBase / 2.0, trackWidth / 2.0);
  public static final double wheelDia = Units.inchesToMeters(4.0);
  public static final double wheelCircum = Units.inchesToMeters(4.0) * Math.PI;
  public static final double driveRatio = 5.9;
  public static final double angleRatio = 18.75/1;
  public static final double speedMultiTeleop = 0.5;
  public static final double speedTurboTeleop = .85;
  public static final double maxSpeed = 4; // meters per second
  public static final double maxAngularSpd = 5;
  public static final double freeSpd = ((6784 / 60) * (wheelDia * Math.PI)) / driveRatio;

      // Swerve Kinematics
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  public static final double driveKs = 0.15;
  public static final double driveKv = 2.0; // 6.4 is the voltage for max speed per module divde by 6.4 plz
  public static final double drivekA = .20; // 6.4 is the voltage for max speed per module
  
  public static final Rotation2d angleOffsetMod0 = Rotation2d.fromRotations(.0352);
  public static final Rotation2d angleOffsetMod1 = Rotation2d.fromRotations(.5263);
  public static final Rotation2d angleOffsetMod2 = Rotation2d.fromRotations(.6401);
  public static final Rotation2d angleOffsetMod3 = Rotation2d.fromRotations(.2003);

  public static final boolean driveStatorEnable = true;
  public static final double driveStatorCurrentLimit = 60;

  public static final double openLoopRamp = 0.1;
  public static final double closedLoopRamp = 0.1;

  // CAN ID's
  public static final int pigeonID = 21;

  public static double elevator_l2 = 23.7;
  public static double elevator_l3 = 62.38;
  public static double elevator_l4  = 124;
  public static double elevator_HP = 11.6;
  public static double elevator_l3_al = 99.2;

  public static double pivot_HP = -47;
  public static double pivot_down = -108.0;
  public static double pivot_90 = -90;

  public static double pivot_up = -20;

  public static double pivot_l3 = -100;


  public static final Mode currentMode = Mode.REAL;

  public static final List<Integer> desiredTagIds = Arrays.asList(6, 7, 8, 9, 10 ,11, 19, 20, 18, 17, 22, 21);

  public static final Map<Integer, Integer> TagToFaceBlue = Map.of(
    18, 0,
    17, 1,
    22, 2, 
    21, 3, 
    20, 4,
    19, 5
  );

  public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
  }

  public static final double driveKp = 2;
  public static final double driveKi = 0.0;
  public static final double driveKd = 0;

  // Angle motor PID Values
  public static final double angleP = 6.5;
  public static final double angleI = 0.0;
  public static final double angleD = 0.0;
  public static final double angleFF = 0.0;
}
