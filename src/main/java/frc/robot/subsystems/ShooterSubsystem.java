// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ErrorConstants;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // Creates new motors
  private TalonFX shooterMotor; 
  private TalonFX hoodMotor;
  private final VelocityVoltage m_flywheelRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage m_hoodRequest = new PositionVoltage(0).withSlot(1);
  private CommandSwerveDrivetrain m_drivetrain ;
  
  private PIDController hoodPID = new PIDController(ShooterConstants.HOOD_P, ShooterConstants.HOOD_I, ShooterConstants.HOOD_D);

  // SIMULATION
  private final double kGearRatio = 1.0;
  private final double kFlywheelMomentOfInertia = 0.5 * Units.lbsToKilograms(3) * Math.pow(Units.inchesToMeters(4), 2);
  private final DCMotor m_gearbox = DCMotor.getKrakenX60(1);
  private final LinearSystem<N1, N1, N1> m_plant = LinearSystemId.createFlywheelSystem(m_gearbox, kFlywheelMomentOfInertia, kGearRatio);
  private final FlywheelSim flywheelSim = new FlywheelSim(m_plant, m_gearbox, 0.01);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {

   shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);
   m_drivetrain = drivetrain;

   if(Utils.isSimulation()){
    var shooterMotorSim = shooterMotor.getSimState();
    shooterMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    shooterMotorSim.setMotorType(MotorType.KrakenX60);
   }

   var flywheelCurrentLimitsConfigs = new CurrentLimitsConfigs();
   flywheelCurrentLimitsConfigs
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(60)
    .withStatorCurrentLimitEnable(true);
   shooterMotor.getConfigurator().apply(flywheelCurrentLimitsConfigs);

   hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
   var slot0Configs = new Slot0Configs();
   slot0Configs.kS = 0;
   slot0Configs.kV = ShooterConstants.FLYWHEEL_V;
   slot0Configs.kP = ShooterConstants.FLYWHEEL_P;
   slot0Configs.kI = ShooterConstants.FLYWHEEL_I;
   slot0Configs.kD = ShooterConstants.FLYWHEEL_D;
   shooterMotor.getConfigurator().apply(slot0Configs);
  
   var slot1Configs = new Slot1Configs();
   slot1Configs.kP = ShooterConstants.HOOD_P;
   slot1Configs.kI = ShooterConstants.HOOD_I;
   slot1Configs.kD = ShooterConstants.HOOD_D;
   hoodMotor.getConfigurator().apply(slot1Configs);

   hoodMotor.setPosition(MathConstants.DegreesToRotations(12) * ShooterConstants.HOOD_GEAR_RATIO);
  
 }

  // Sets the shooter motor to a speed
  public void setFlywheelMotor(double speed) {
    shooterMotor.set(speed);
  }

  public void setHoodMotor(double speed) {
    hoodMotor.set(speed);
  }

  public void setFlywheelToSpeed(double RPM){
    shooterMotor.setControl(m_flywheelRequest.withVelocity(MathConstants.RPMtoRPS(RPM))
    .withFeedForward(0.00));
  }

  public boolean isFlywheelNearSetpoint(double RPM) {
    return MathUtil.isNear(RPM, MathConstants.RPStoRPM(shooterMotor.getVelocity().getValueAsDouble()), ErrorConstants.TURRET_RPM_TOLERANCE);
  }

  public void setHoodMotorPosition(double angle) {
    //SmartDashboard.putNumber("Actual Hood Setpoint", angle);
    hoodMotor.setControl(m_hoodRequest.withPosition(MathConstants.DegreesToRotations(angle) * ShooterConstants.HOOD_GEAR_RATIO));
  }

  public void setHoodMotorPositionNEW(double angle) {
    hoodMotor.set(hoodPID.calculate(hoodMotor.getPosition().getValueAsDouble() / ShooterConstants.HOOD_GEAR_RATIO, MathConstants.DegreesToRotations(angle)));
  }

  public double distanceToFlywheelSpeed(double distance) {
    return 1013.63192 * (Math.pow(1.31539, distance));//1.21174=power
  }

  public double distanceToHoodAngleDegrees(double distance) {
    return 0.82021 * distance + 10.25; // 9.56449 * Math.pow(1.09654, distance) <---- this was to test exponential regression on the hood because it was hitting the ceiling from 21 feet
  }

  public Command shootFromShootPose(){
  
    return Commands.run(() -> {
      if (MathUtil.isNear(PoseConstants.BLUE_SHOOT_POSE.getX(), m_drivetrain.getPose2d().getX() , 0.2 )
      && MathUtil.isNear(PoseConstants.BLUE_SHOOT_POSE.getY(), m_drivetrain.getPose2d().getY() , 0.2) && m_drivetrain.isAllianceBlue())
      {
        setFlywheelToSpeed(1750);
      }

    }, this)
    .beforeStarting(Commands.run(() -> {
      if (MathUtil.isNear(PoseConstants.RED_SHOOT_POSE.getX(), m_drivetrain.getPose2d().getX() , 0.2 )
      && MathUtil.isNear(PoseConstants.RED_SHOOT_POSE.getY(), m_drivetrain.getPose2d().getY() , 0.2) && m_drivetrain.isAllianceRed())
      {
        setFlywheelToSpeed(1750);
      }
      
    }, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Measured Flywheel Speed", MathConstants.RPStoRPM(shooterMotor.getVelocity().getValueAsDouble()));
    //SmartDashboard.putNumber("Measured Hood Angle", (hoodMotor.getPosition().getValueAsDouble() / ShooterConstants.HOOD_GEAR_RATIO * 360));

    
    // setHoodMotorPosition(SmartDashboard.getNumber("Hood Setpoint", 12));
    // setFlywheelToSpeed(SmartDashboard.getNumber("Flywheel Setpoint", 0));
    

    if(m_drivetrain.calculateTurretFieldPositionX() > PoseConstants.BLUE_ALLIANCE_ZONE_X && m_drivetrain.calculateTurretFieldPositionX() < PoseConstants.RED_ALLIANCE_ZONE_X) {
      setHoodMotorPosition(ShooterConstants.HOOD_MAX_ANGLE);
    }
    else {
      setHoodMotorPosition(ShooterConstants.HOOD_HOME_ANGLE);
    } 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    var shooterMotorSim = shooterMotor.getSimState();
    shooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = shooterMotorSim.getMotorVoltageMeasure();

    flywheelSim.setInputVoltage(motorVoltage.in(Volts));
    flywheelSim.update(0.020);

    shooterMotorSim.setRotorVelocity(flywheelSim.getAngularVelocity().times(kGearRatio));
  }

  public double getSimulatedMotorCurrent() {
    return flywheelSim.getCurrentDrawAmps();
  }
}
