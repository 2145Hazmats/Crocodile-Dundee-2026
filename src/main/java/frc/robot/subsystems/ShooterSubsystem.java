// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // Creates new motors
  private TalonFX shooterMotor; 
  private TalonFX kickerMotor;
  private PIDController flywheelPID;
  private TalonFX hoodMotor;
  private PIDController hoodPID;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
   shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);
   kickerMotor = new TalonFX(ShooterConstants.KICKER_MOTOR_ID);
   flywheelPID = new PIDController(ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D); 
   hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
   hoodPID = new PIDController(ShooterConstants.HOOD_P, ShooterConstants.HOOD_I, ShooterConstants.HOOD_D);
 }

  // Sets the shooter motor to a speed
  public void setShooterMotor(double speed) {
    shooterMotor.set(speed);
  }

  // Sets the kicker motor to a speed
  public void setKickerMotor(double speed) {
    kickerMotor.set(speed);
  }

  public void setHoodMotor(double speed){
    hoodMotor.set(speed);
  }

  public Command hoodUpAndDown(double position){
    return Commands.run(() -> hoodMotor.set(hoodPID.calculate(hoodMotor.getPosition().getValueAsDouble(), position)));
  }
  

  public Command flywheelRevUp(double setpoint) {
    return Commands.run(()-> shooterMotor.set(flywheelPID.calculate(shooterMotor.getVelocity().getValueAsDouble() * 60, setpoint)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
