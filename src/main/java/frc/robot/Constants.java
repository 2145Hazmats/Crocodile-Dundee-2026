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
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

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
    public static final int P1_CONTROLLER_PORT = 0;
    public static final int P2_CONTROLLER_PORT = 1;
    public static final int EVERYTHING_CONTROLLER_PORT = 2;
    public static final int TESTING_CONTROLLER_PORT = 3;
  }

  public static class PathPlannerConstants {
    // PID Autobuilder
    public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ROTATIONAL_PID = new PIDConstants(5, 0, 0);

    public static final double MAX_VELOCITY_MPS = 3.5;
    public static final double MAX_ACCELERATION_MPS = 3;
    public static final double MAX_ANGULAR_ACCELERATION_RAD = Units.degreesToRadians(75);
    public static final double MAX_ANGULAR_VELOCITY_RAD = Units.degreesToRadians(360);
    public static final double NOMINAL_VOLTAGE_VOLTS = 11.5;
  }
  
  public static class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 24;
    public static final double CLIMB_UP_POSITION = 5;
    public static final double CLIMB_HOME_POSITION = 0;
  }
  
  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 22;
    public static final double TURRET_P = 0.4;
    public static final double TURRET_I = 0.00000;
    public static final double TURRET_D = 0.01;
    public static final double TURRET_GEAR_RATIO = 14.285714285714286;
    public static final double TURRET_STARTING_ANGLE = 0;
  }
  
  public static class SpindexerConstants {
    public static final int SPINDEXER_MOTOR_ID = 20;
  }
    
  public static class IntakeConstants {
    public static final int ACTUATOR_INTAKE_MOTOR_ID = 30;
    public static final int INTAKING_MOTOR_ID = 31;
    public static final double ACTUATOR_ALL_THE_WAY_IN = 0;
    public static final double ACTUATOR_HOME_POSITION = -1;   // TODO: Set home and down positions for the intake
    public static final double ACTUATOR_DOWN_POSITION = -30.19;   // <-----------------------------------------------
    public static final double INTAKE_MOTOR_SPEED = 1;
    public static final double ACTUATOR_P = 0.25;
    public static final double ACTUATOR_I = 0.0000;
    public static final double ACTUATOR_D = 0.0000;
   
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 29;
    public static final int FEEDER_MOTOR_ID = 21;
    public static final double FLYWHEEL_P = 0.6
    ;
    public static final double FLYWHEEL_I = 0.00000;
    public static final double FLYWHEEL_D = 0.0;
    public static final double FLYWHEEL_V = 0.15;
    public static final double FLYWHEEL_RPM_SETPOINT = 2500; 

    public static final int HOOD_MOTOR_ID = 28;
    public static final double HOOD_P = 2;
    public static final double HOOD_I = 0.00000;
    public static final double HOOD_D = 0.00000;
    public static final double HOOD_GEAR_RATIO = 71.5714;
    public static final double HOOD_MAX_ANGLE = 25;
    public static final double HOOD_HOME_ANGLE = 12;
  }

  public static class PoseConstants {
    public static final double[] BLUE_ALLIANCE_HUB_LOCATION = {4.625, 4.035};
    public static final double[] RED_ALLIANCE_HUB_LOCATION = {11.915, 4.035};

    public static final double[] BLUE_ALLIANCE_RIGHT_CORNER = {1, 1};
    public static final double[] BLUE_ALLIANCE_LEFT_CORNER = {1, 7.069};
    public static final double[] RED_ALLIANCE_RIGHT_CORNER = {15.541, 7.069};
    public static final double[] RED_ALLIANCE_LEFT_CORNER = {15.541, 1};
    
    public static final double RED_ALLIANCE_ZONE_X = 11.913;
    public static final double CENTER_FIELD_Y = 4.035; 
    public static final double BLUE_ALLIANCE_ZONE_X = 4.629;

    public static final Pose2d BLUE_SHOOT_POSE = new Pose2d(1.590, 4.02, new Rotation2d(0));
    public static final Pose2d RED_SHOOT_POSE = new Pose2d(14.45, 4.02, new Rotation2d(180));
  }
  
  public static class VisionConstants{
    public static final Transform3d FRONT_CAMERA_POSITION = 
    new Transform3d(Units.inchesToMeters(8.804), Units.inchesToMeters(-12.735), Units.inchesToMeters(9.407),
    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(10), Units.degreesToRadians(0)));
    public static final double AMBIGUITY_RATIO_CUTOFF = 0.2;
    public static final Transform3d SIDE_CAMERA_POSITION = 
    new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0),
    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));
  }
  
  public static class ErrorConstants {
    public static final double ACTUATOR_HOME_ERROR = 3.0;
    public static final double TURRET_RPM_TOLERANCE = 100;
  }

  public static class MathConstants {
    public static double DegreesToRotations(double angle) {
      return angle / 360;
    }

    public static double RadiansToRotations(double angle) {
      return angle / 2 * Math.PI;
    }

    public static double RPMtoRPS(double RPM) {
      return RPM / 60;
    }

    public static double RPStoRPM(double RPS) {
      return RPS * 60;
    }
  }
}


