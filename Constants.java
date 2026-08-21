// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class SwerveConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_LEFT_CANCODER_ID = 2;

    public static final int FRONT_RIGHT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_STEER_ID = 4;
    public static final int FRONT_RIGHT_CANCODER_ID = 5;

    public static final int BACK_LEFT_DRIVE_ID = 6;
    public static final int BACK_LEFT_STEER_ID = 7;
    public static final int BACK_LEFT_CANCODER_ID = 8;

    public static final int BACK_RIGHT_DRIVE_ID = 9;
    public static final int BACK_RIGHT_STEER_ID = 10;
    public static final int BACK_RIGHT_CANCODER_ID = 11;

    //motorların port idleri

    public static final int PIGEON_2_ID = 12;

    public static final int TURRET_MOTOR_ID = 13;

    public static final int SHOOTER_MOTOR_ID = 14;

    //drivetrain

    public static final double STEER_kP = 4;
    public static final double STEER_kD = 0;


    public static final double DRIVE_kP = 3.3;
    public static final double DRIVE_kD = 0;

    public static final double STEER_GEAR_RATIO = 287.0 / 11.0;
    public static final double STEER_SENSOR_RATIO = 1;

    public static final double TEKERLEK_CAPI = 10.16;
    public static final double TEKERLEK_YARICAPI = 0.0508;
    public static final double TEKERLEK_CEVRESI = 0.319;
    public static final double DRIVE_GEAR_RATIO = 7.03;
    public static final double MAX_SPEED = 4.54;
    public static final double MAX_ANGULAR_SPEED = 2;
    //bu radyan bu ard


    //turn to angle command 

    public static final double TURN_kP = 0.25;
    public static final double TURN_kI  = 0;
    public static final double TURN_kD = 0;

    //turret subsystem 

    public static final double LEFT_ANGLE = -90.0;
    public static final double RIGHT_ANGLE = 90.0;
    public static final double MIN_ANGLE = -120.0;
    public static final double MAX_ANGLE = 120.0;
    public static final double ZERO_TURRET = 0;
    public static final double TURRET_kP = 0.02;
    public static final double TURRET_kI = 0.0;
    public static final double TURRET_KD = 0.0;
    public static final double TURRET_kS = 0.1;
    public static final double TURRET_kV = 0.1;
    public static final double BATTERY = 12.0;

    //shooter subsystem

    public static final double SHOOTER_kP = 0.6;
    public static final double SHOOTER_kI = 0.0;
    public static final double SHOOTER_kD = 0.0;
    public static final double SHOOTER_kS = 0.4;
    public static final double SHOOTER_kV = 0.3;


    //otonom canısı

    public static final double AUTO_TRANS_kP = 0.02;
    public static final double AUTO_TRANS_kI = 0.0;
    public static final double AUTO_TRANS_kD = 0.0;

    public static final double AUTO_ROT_kP = 0.02;
    public static final double AUTO_ROT_kI = 0.0;
    public static final double AUTO_ROT_kD = 0.0;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
