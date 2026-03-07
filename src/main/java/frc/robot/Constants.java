// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int EVERYTHING_CONTROLLER_PORT = 2;
  }

  public static class PathPlannerConstants {
    // PID Autobuilder
    public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ROTATIONAL_PID = new PIDConstants(5, 0, 0);
  }
  
  public static class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 24;
    public static final double CLIMB_UP_POSITION = 5;
    public static final double CLIMB_HOME_POSITION = 0;
    public static final double SERVO_LOCK_POSITION = 5;
    public static final double SERVO_UNLOCK_POSITION = 0;
    public static final int SERVO_CHANNEL = 1;
  }
  
  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 22;
    public static final double TURRET_P = 0.1;
    public static final double TURRET_I = 0.00000;
    public static final double TURRET_D = 0.001;
    public static final double TURRET_GEAR_RATIO = 14.285714285714285714;
  }
  
  public static class SpindexerConstants {
    public static final int SPINDEXER_MOTOR_ID = 20;
  }
    
  public static class IntakeConstants {
    public static final int ACTUATOR_INTAKE_MOTOR_ID = 30;
    public static final int INTAKING_MOTOR_ID = 31;
    public static final double ACTUATOR_HOME_POSITION = 18.59423828125;
    public static final double ACTUATOR_DOWN_POSITION = -16.13037109375;
    public static final double INTAKE_MOTOR_SPEED = 0.3;
    public static final double ACTUATOR_DOWN_P = 0.1;
    public static final double ACTUATOR_DOWN_I = 0.0000;
    public static final double ACTUATOR_DOWN_D = 0.0000;
    public static final double ACTUATOR_HOME_P = 0.15;
    public static final double ACTUATOR_HOME_I = 0.0;
    public static final double ACTUATOR_HOME_D = 0.0;
   
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 40;
    public static final int KICKER_MOTOR_ID = 21;
    public static final double FLYWHEEL_P = 0.00001;
    public static final double FLYWHEEL_I = 0.00000;
    public static final double FLYWHEEL_D = 0.00000;
    public static final double FLYWHEEL_RPM_SETPOINT = 2000;
  }

  public static class PoseConstants {
    public static final double[] BLUE_ALLIANCE_HUB_LOCATION = {4.62534, 4.034536};
    public static final double[] RED_ALLIANCE_HUB_LOCATION = {11.91514, 4.034536};
  }
  public static class VisionConstants{
    public static final Transform3d FRONT_CAMERA_POSITION = 
    new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0), 
    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(10), Units.degreesToRadians(0)));
    public static final double AMBIGUITY_RATIO_CUTOFF = 0.2;
  }
  
  public static class ErrorConstants {
    public static final double ACTUATOR_HOME_ERROR = 3.0;
  }
}


