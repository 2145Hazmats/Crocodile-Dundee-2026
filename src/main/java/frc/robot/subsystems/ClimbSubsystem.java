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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);
  private PositionVoltage m_climbRequest = new PositionVoltage(0).withSlot(0);

  /** Creates a new Climbsubsystem. */
  public ClimbSubsystem() {
    var climbCurrentLimitsConfigs = new CurrentLimitsConfigs();
   climbCurrentLimitsConfigs
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(60)
    .withStatorCurrentLimitEnable(true);
   climbMotor.getConfigurator().apply(climbCurrentLimitsConfigs);

   var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.01;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    climbMotor.getConfigurator().apply(slot0Configs);

  }

  public void setMotor(double speed){
    climbMotor.set(speed);
  }



  public boolean climbIsClose(double position){
    return MathUtil.isNear(position, climbMotor.getPosition().getValueAsDouble(), .1);
  }

  public Command moveClimbToPosition(double position){
    return Commands.run(()-> climbMotor.setControl(m_climbRequest.withPosition(position)), this)
    .until(() -> climbIsClose(position));
  }
 // public Command climbRelease(){
  //  return Commands.run(()-> moveServoToPosition(ClimbConstants.SERVO_UNLOCK_POSITION), this);
 // } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}