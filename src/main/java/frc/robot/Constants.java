// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 0;
  }
  
  public static class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 16;
    public static final int SERVO_CHANNEL = 1;
  }
  
  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 13;
    public static final double TURRET_P = 0.000001;
    public static final double TURRET_I = 0.00000;
    public static final double TURRET_D = 0.000000;
  }
  
  public static class SpindexerConstants {
    public static final int SPINDEXER_MOTOR_ID = 12;
  }
    
  public static class IntakeConstants {
    public static final int ACTUATOR_INTAKE_MOTOR_ID = 9;
    public static final int INTAKING_MOTOR_ID = 11;
    public static final double ACTUATOR_HOME_POSITION = 0.0;
    public static final double ACTUATOR_DOWN_POSITION = 5.0;
    public static final double INTAKE_MOTOR_SPEED = 0.5;
    public static final double ACTUATOR_P = 0.00001;
    public static final double ACTUATOR_I = 0.0000;
    public static final double ACTUATOR_D = 0.0000;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 14;
    public static final int KICKER_MOTOR_ID = 15;
    public static final double FLYWHEEL_P = 0.00001;
    public static final double FLYWHEEL_I = 0.00000;
    public static final double FLYWHEEL_D = 0.00000;
  }
  
  
  public static class ErrorConstants {
    public static final double ACTUATOR_HOME_ERROR = 2;
  }
}

