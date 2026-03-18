// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private TalonFX turretMotor;
  private PIDController turretPID;
  private final PositionVoltage m_turretRequest = new PositionVoltage(0).withSlot(0);

  private CommandSwerveDrivetrain m_drivetrain;

  /** Creates a new Turret. */
  public TurretSubsystem(CommandSwerveDrivetrain drivetrain) {
   turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);

   var turretCurrentLimitsConfig = new CurrentLimitsConfigs();
   turretCurrentLimitsConfig.withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);
   turretMotor.getConfigurator().apply(turretCurrentLimitsConfig);

   turretPID = new PIDController(TurretConstants.TURRET_P,TurretConstants.TURRET_I, TurretConstants.TURRET_D);
   var slot0Configs = new Slot0Configs();
   slot0Configs.kS = 0;
   slot0Configs.kP = TurretConstants.TURRET_P;
   slot0Configs.kI = TurretConstants.TURRET_I;
   slot0Configs.kD = TurretConstants.TURRET_D;
   turretMotor.getConfigurator().apply(slot0Configs);

   turretMotor.setPosition(MathConstants.DegreesToRotations(TurretConstants.TURRET_STARTING_ANGLE));

   m_drivetrain = drivetrain;
  }

  public void setMotor(double speed) {
    turretMotor.set(speed);
  }
  //Turret Command that uses PID, clamped between setpoint of -90 and 90 degrees
  public Command turnTurretToAngle(DoubleSupplier angleToPointTo) {
    return Commands.run(
      () -> {
        double setpoint = 0;
        if(m_drivetrain.isAllianceBlue()) {
           setpoint = -angleToPointTo.getAsDouble() + m_drivetrain.calculateAngleToFieldPosition(m_drivetrain.getPose2d().getX(), m_drivetrain.getPose2d().getY());
        }
        else if (m_drivetrain.isAllianceRed()) {
          setpoint = -angleToPointTo.getAsDouble() + m_drivetrain.calculateAngleToFieldPosition(m_drivetrain.getPose2d().getX(), m_drivetrain.getPose2d().getY());
        }

        setMotor(turretPID.calculate(turretMotor.getPosition().getValueAsDouble() / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2, MathUtil.clamp(setpoint, Math.toRadians(-90), Math.toRadians(90))));
      },
      this
    ).finallyDo(
      () -> setMotor(0)
    );
  }

  public Command turnTurretToAngleNew(DoubleSupplier angleToPointTo){
    return Commands.run(
      ()->{
        double setpoint = -angleToPointTo.getAsDouble() + m_drivetrain.calculateAngleToFieldPosition(m_drivetrain.getPose2d().getX(), m_drivetrain.getPose2d().getY());
        turretMotor.setControl(m_turretRequest.withPosition(MathConstants.RadiansToRotations(MathUtil.clamp(setpoint, Math.toRadians(-90), Math.toRadians(90)))));
      });
  }

 

  public double calculateTurretFieldPositionX(){
    
    //this is the original position of the drivetrain from the center of the robot
    double drivetrainFieldPositionX = m_drivetrain.getPose2d().getX();
    
    // This vector is the position of the turret in the robot  on the X axis
    // Compensates for the initial angle of the turret relative to the robot
    double positionOfTurretX = Math.cos(m_drivetrain.getPose2d().getRotation().getRadians()) * TurretConstants.TURRET_MAGNITUDE_FROM_CENTER;
    
    // Adding the two together gets you the turret's position on the field
    double turretFieldPositionX = drivetrainFieldPositionX + positionOfTurretX;
    
    return turretFieldPositionX;
  }

  public double calculateTurretFieldPositionY(){
    //this is the original position of the drivetrain from the center of the robot
    double drivetrainFieldPositionY = m_drivetrain.getPose2d().getY();

    // This vector is the position of the turret in the robot  on the X axis
    // Compensates for the initial angle of the turret relative to the robot
    double positionOfTurretY = Math.sin(m_drivetrain.getPose2d().getRotation().getRadians()) * TurretConstants.TURRET_MAGNITUDE_FROM_CENTER;
    
    // Adding the two together gets you the turret's position on the field
    double turretFieldPositionY = drivetrainFieldPositionY + positionOfTurretY;

    return turretFieldPositionY;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Angle", Units.radiansToDegrees(turretMotor.getPosition().getValueAsDouble()
     / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2));
  }
}
