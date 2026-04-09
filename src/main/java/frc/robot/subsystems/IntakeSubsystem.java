// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ErrorConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX actuatorMotor;
  private TalonFX intakingMotor;
  private final PositionVoltage m_actuatorRequest = new PositionVoltage(0).withSlot(0);
  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    actuatorMotor = new TalonFX(IntakeConstants.ACTUATOR_INTAKE_MOTOR_ID);
    intakingMotor = new TalonFX(IntakeConstants.INTAKING_MOTOR_ID);

    var intakingCurrentLimitConfig = new CurrentLimitsConfigs();
    intakingCurrentLimitConfig.withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true);
    intakingCurrentLimitConfig.withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true);
    intakingMotor.getConfigurator().apply(intakingCurrentLimitConfig);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kG = 0.4;
    slot0Configs.kP = IntakeConstants.ACTUATOR_P;
    slot0Configs.kI = IntakeConstants.ACTUATOR_I;
    slot0Configs.kD = IntakeConstants.ACTUATOR_D;
    actuatorMotor.getConfigurator().apply(slot0Configs);

    resetIntakePosition();
  }
  
  public void resetIntakePosition() {
    actuatorMotor.setPosition(0);
  }


  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("GroundIntakePosition", getActuatorPosition());
    // This method will be called once per scheduler run
  }

  public void setActuatorMotor(double speed){
    actuatorMotor.set(speed);
  }

  public void setIntakingMotor(double speed){
    intakingMotor.set(speed);
  }

  public double getActuatorPosition() {
    return actuatorMotor.getPosition().getValueAsDouble();
  }

  public Command setIntakePosition(double position){
     return Commands.runOnce(() -> actuatorMotor.setControl(m_actuatorRequest.withPosition(position)), this);
  }
  
  // public Command autoIntakeHOME() {
  //   return Commands.run(() -> {
  //     setIntakePosition(IntakeConstants.ACTUATOR_HOME_POSITION);
  //     setIntakingMotor(0);
  //   }, this);
  // }

  // public Command autoIntakeUnjam() {
  //   return Commands.run(() -> setIntakingMotor(IntakeConstants.INTAKE_MOTOR_SPEED)).withTimeout(2);
  // }
}


