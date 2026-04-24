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
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(6 * TurretConstants.TURRET_GEAR_RATIO, 12 * TurretConstants.TURRET_GEAR_RATIO));

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
        
        setMotor(turretPID.calculate(turretMotor.getPosition().getValueAsDouble() / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2, MathUtil.clamp(goal, Math.toRadians(-93), Math.toRadians(93))));
      },
      this
    ).finallyDo(
      () -> setMotor(0)
    );
  }

  public double angularVelocityToTurretOffset(){
    return -m_drivetrain.getFilteredAngularVelocity() * 0.15;
  }

  public Command turnTurretToAngleProfiled(DoubleSupplier angleToPointTo) {
    return Commands.run(
      () -> {
        
        double angle = -angleToPointTo.getAsDouble() + m_drivetrain.getPose2d().getRotation().getRadians() /*+ angularVelocityToTurretOffset()*/;

        if(m_drivetrain.getPose2d().getX() > m_drivetrain.getTargetPose()[0]) { 
            angle += Math.PI;
        }
        
        if(angle > Math.PI) {
          angle -= (Math.PI * 2);
        }
        else if(angle < -Math.PI) {
          angle += (Math.PI * 2);
        }

        TrapezoidProfile.State goal = new TrapezoidProfile.State(MathUtil.clamp(angle, Math.toRadians(-95), Math.toRadians(95)) / (Math.PI * 2) * TurretConstants.TURRET_GEAR_RATIO, 0);

        previousProfiledReference = m_profile.calculate(0.020, previousProfiledReference, goal);

        m_turretRequest.Position = previousProfiledReference.position;
        m_turretRequest.Velocity = previousProfiledReference.velocity;

        turretMotor.setControl(m_turretRequest);

        SmartDashboard.putNumber("Turret Goal", Units.radiansToDegrees(goal.position));
        SmartDashboard.putNumber("Desired Turret Angle", angle);
        //SmartDashboard.putNumber("Actual Turret Setpoint", previousProfiledReference.position);
        //SmartDashboard.putNumber("Rotation of Robot", m_drivetrain.getPose2d().getRotation().getDegrees());
        //SmartDashboard.putNumber("Angular Velocity", Units.radiansToDegrees(m_drivetrain.getFilteredAngularVelocity()));

      }, this)
      .finallyDo(
        () -> setMotor(0)
      );
  }

   public double getDistanceToTarget(double targetX, double targetY) {
        double robotX = m_drivetrain.calculateTurretFieldPositionX();
        double robotY = m_drivetrain.calculateTurretFieldPositionY();

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
    
    

    turretPos[0] = m_drivetrain.calculateTurretFieldPositionX();
    turretPos[1] = m_drivetrain.calculateTurretFieldPositionY();

    turretField.setRobotPose(new Pose2d(turretPos[0], turretPos[1], new Rotation2d()));
  }
}
