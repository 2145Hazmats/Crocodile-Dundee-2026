// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private Servo lockServo = new Servo(ClimbConstants.SERVO_CHANNEL);
  private PIDController climbPID = new PIDController(.001, 0, 0);
  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {}

  public void setMotor(double speed){
    climbMotor.set(speed);
  }

  public void setServo(double angle) {
    lockServo.setAngle(angle);
  }

  public boolean climbIsClose(double position){
    return MathUtil.isNear(position, climbMotor.getPosition().getValueAsDouble(), .1);
  }

  public Command moveClimbToPosition(double position){
    return Commands.run(()-> climbMotor.set(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), 
    position)), this)
    .until(() -> climbIsClose(position));
  }
  public Command moveServoToPosition(double angle){
    return Commands.run(()-> setServo(angle));
  }

  public Command climbDown(){
    return new SequentialCommandGroup(
      moveClimbToPosition(ClimbConstants.CLIMB_UP_POSITION),
      moveServoToPosition(ClimbConstants.SERVO_UNLOCK_POSITION),
      moveClimbToPosition(ClimbConstants.CLIMB_HOME_POSITION));
  }

  public Command climbUp(){
    return new SequentialCommandGroup(
      moveServoToPosition(ClimbConstants.SERVO_LOCK_POSITION),
      moveClimbToPosition(ClimbConstants.CLIMB_UP_POSITION),
      moveClimbToPosition(ClimbConstants.CLIMB_HOME_POSITION));
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