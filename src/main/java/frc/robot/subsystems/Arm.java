// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm extends SubsystemBase {

  private final CANSparkMax armMotor;

  private SparkMaxPIDController armPIDController;
  private RelativeEncoder armEncoder;
  
  double kP = Constants.ArmConstants.armKP,
    kI = Constants.ArmConstants.armKI,
    kD = Constants.ArmConstants.armKD,
    kIz = Constants.ArmConstants.armKIz,
    kFF = Constants.ArmConstants.armKFF, 
    kMinOutput = Constants.ArmConstants.armKMinOutput,
    kMaxOutput = Constants.ArmConstants.armKMaxOutput,
    minVel = Constants.ArmConstants.armMinVel,
    maxVel = Constants.ArmConstants.armMaxVel,
    maxAcc = Constants.ArmConstants.armMaxAcc,
    allowedErr = Constants.ArmConstants.armAllowedErr;

  
  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    armMotor.setSmartCurrentLimit(30);
    armMotor.setIdleMode(IdleMode.kBrake);

    //armMotor.setSoftLimit(SoftLimitDirection.kForward, 5);
    //armMotor.setSoftLimit(SoftLimitDirection.kReverse, -80);

    //armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    //armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // initialze PID controller and encoder objects
    armPIDController = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();
    

    // set PID coefficients
    armPIDController.setP(kP);
    armPIDController.setI(kI);
    armPIDController.setD(kD);
    armPIDController.setIZone(kIz);
    armPIDController.setFF(kFF);
    armPIDController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    armPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    armPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    armPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    armPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

  }

  public void setPosition(double targetPosition){
    armPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);    
  }

  public void manualArm(double speed){
    armMotor.set(speed);
  }

  public double getOutputCurrent(){
    return armMotor.getOutputCurrent();
  }

  public double getPosition(){
    return armEncoder.getPosition();
  }

  public void resetEncoder(){
    armMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Current", getOutputCurrent());
    SmartDashboard.putNumber("Arm Position", getPosition());
  }
}
