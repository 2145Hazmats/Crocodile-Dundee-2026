// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // Creates new motors
  private TalonFX shooterMotor; 
  private TalonFX kickerMotor;
  private PIDController flywheelPID;
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
   shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);
   kickerMotor = new TalonFX(ShooterConstants.KICKER_MOTOR_ID);
   flywheelPID = new PIDController(ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D); 
 }

  // Sets the shooter motor to a speed
  public void setShooterMotor(double speed) {
    shooterMotor.set(speed);
  }

  // Sets the kicker motor to a speed
  public void setKickerMotor(double speed) {
    kickerMotor.set(speed);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
