// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private Field2d turretField = new Field2d();

  private TalonFX turretMotor;
  private PIDController turretPID;
  private final PositionVoltage m_turretRequest = new PositionVoltage(0).withSlot(0);
  private TrapezoidProfile.State previousProfiledReference;
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3 * TurretConstants.TURRET_GEAR_RATIO, 18 * TurretConstants.TURRET_GEAR_RATIO));

  private CommandSwerveDrivetrain m_drivetrain;

  private double[] turretPos = new double[2];
  /** Creates a new Turret. */
  public TurretSubsystem(CommandSwerveDrivetrain drivetrain) {
   turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);

   var turretCurrentLimitsConfig = new CurrentLimitsConfigs();
   turretCurrentLimitsConfig.withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);
   turretMotor.getConfigurator().apply(turretCurrentLimitsConfig);

   turretPID = new PIDController(TurretConstants.TURRET_P,TurretConstants.TURRET_I, TurretConstants.TURRET_D);


   var slot0Configs = new Slot0Configs();
   slot0Configs.kS = 0.3;
   slot0Configs.kP = TurretConstants.TURRET_P;
   slot0Configs.kI = TurretConstants.TURRET_I;
   slot0Configs.kD = TurretConstants.TURRET_D;
   turretMotor.getConfigurator().apply(slot0Configs);

   turretMotor.setPosition(MathConstants.DegreesToRotations(TurretConstants.TURRET_STARTING_ANGLE));
   previousProfiledReference = new State(turretMotor.getPosition().getValueAsDouble(), turretMotor.getVelocity().getValueAsDouble());

   SmartDashboard.putData("Turret Field", turretField);

   m_drivetrain = drivetrain;
  }

  public void setMotor(double speed) {
    turretMotor.set(speed);
  }
  //Turret Command that uses PID, clamped between setpoint of -90 and 90 degrees
  public Command turnTurretToAngle(DoubleSupplier angleToPointTo) {
    return Commands.run(
      () -> {
        double goal = -angleToPointTo.getAsDouble() + m_drivetrain.getPose2d().getRotation().getRadians();
        
        setMotor(turretPID.calculate(turretMotor.getPosition().getValueAsDouble() / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2, MathUtil.clamp(goal, Math.toRadians(-95), Math.toRadians(93))));
      },
      this
    ).finallyDo(
      () -> setMotor(0)
    );
  }

  public double angularVelocityToTurretOffset(){
    return -m_drivetrain.getFilteredAngularVelocity() * 0.15;
  }

  public Command turnTurretToAngleProfiled(DoubleSupplier angleToPointTo){
    return Commands.run(
      ()->{
        
        double angle = -angleToPointTo.getAsDouble() + m_drivetrain.getPose2d().getRotation().getRadians() /*+ angularVelocityToTurretOffset()*/;


        TrapezoidProfile.State goal = new TrapezoidProfile.State(MathUtil.clamp(angle, Math.toRadians(-93), Math.toRadians(93)) / (Math.PI * 2) * TurretConstants.TURRET_GEAR_RATIO, 0);

        previousProfiledReference = m_profile.calculate(0.020, previousProfiledReference, goal);
        SmartDashboard.putNumber("Turret Goal", goal.position);
        SmartDashboard.putNumber("Actual Turret Setpoint", previousProfiledReference.position);
        SmartDashboard.putNumber("Rotation of Robot", m_drivetrain.getPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("Angular Velocity", Units.radiansToDegrees(m_drivetrain.getFilteredAngularVelocity()));
        m_turretRequest.Position = previousProfiledReference.position;
        m_turretRequest.Velocity = previousProfiledReference.velocity;

        turretMotor.setControl(m_turretRequest);
      }, this)
      .finallyDo(
        () -> setMotor(0)
      );
  }
 

  public double calculateTurretFieldPositionX(){
    
    //this is the original position of the drivetrain from the center of the robot
    double drivetrainFieldPositionX = m_drivetrain.getPose2d().getX();
    
    // This vector is the position of the turret in the robot  on the X axis
    // Compensates for the initial angle of the turret relative to the robot
    double robotRelativePositionOfTurretX = Math.cos(m_drivetrain.getPose2d().getRotation().getRadians()) * TurretConstants.TURRET_DISTANCE_FROM_CENTER;
    
    // Adding the two together gets you the turret's position on the field
    double turretFieldPositionX = drivetrainFieldPositionX + robotRelativePositionOfTurretX;
    turretPos[0] = turretFieldPositionX;
    
    return turretFieldPositionX;
  }

  public double calculateTurretFieldPositionY(){
    //this is the original position of the drivetrain from the center of the robot
    double drivetrainFieldPositionY = m_drivetrain.getPose2d().getY();

    // This vector is the position of the turret in the robot  on the X axis
    // Compensates for the initial angle of the turret relative to the robot
    double robotRelativePositionOfTurretY = Math.sin(m_drivetrain.getPose2d().getRotation().getRadians()) * TurretConstants.TURRET_DISTANCE_FROM_CENTER;
  
    // Adding the two together gets you the turret's position on the field
    double turretFieldPositionY = drivetrainFieldPositionY + robotRelativePositionOfTurretY;

    turretPos[1] = turretFieldPositionY;

    return turretFieldPositionY;
  }

   public double getDistanceToTarget(double targetX, double targetY) {
        double robotX = calculateTurretFieldPositionX();
        double robotY = calculateTurretFieldPositionY();

        double relativeX = robotX - targetX;
        double relativeY = robotY - targetY;

        double distance = Math.sqrt(relativeX * relativeX + relativeY * relativeY);
        return Units.metersToFeet(distance);
    }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Angle", Units.radiansToDegrees(turretMotor.getPosition().getValueAsDouble()
     / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2));
    
    SmartDashboard.putNumber("Turret Distance To Hub", getDistanceToTarget(PoseConstants.BLUE_ALLIANCE_HUB_LOCATION[0], PoseConstants.BLUE_ALLIANCE_HUB_LOCATION[1]));

    turretField.setRobotPose(new Pose2d(turretPos[0], turretPos[1], new Rotation2d()));
  }
}
