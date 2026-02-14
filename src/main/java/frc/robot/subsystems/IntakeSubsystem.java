// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ErrorConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX actuatorMotor;
  private TalonFX intakingMotor;
  private PIDController actuatorPID;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    actuatorMotor = new TalonFX(IntakeConstants.ACTUATOR_INTAKE_MOTOR_ID);
    intakingMotor = new TalonFX(IntakeConstants.INTAKING_MOTOR_ID);
    actuatorPID = new PIDController(Constants.PIDActuatorConstants.ACTUATOR_P, Constants.PIDActuatorConstants.ACTUATOR_I
    , Constants.PIDActuatorConstants.ACTUATOR_D);
  }
  public boolean isIntakeInRobot(){
    return MathUtil.isNear(IntakeConstants.ACTUATOR_HOME_POSITION, getActuatorPosition(), ErrorConstants.ACTUATOR_HOME_ERROR );
  }

  @Override
  public void periodic() {
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
  public Command MoveActuatorPosition(double position) {
    return Commands.run(() -> actuatorMotor.set(actuatorPID.calculate(actuatorMotor.getPosition().getValueAsDouble(), position)));
  }
}

