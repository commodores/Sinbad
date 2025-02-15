// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final CANSparkMax wristMotor;

  private SparkMaxPIDController wristPIDController;
  private RelativeEncoder wristEncoder;
  private final DigitalInput reverseLimitSwitch;

  double kP = Constants.WristConstants.wristKP,
    kI = Constants.WristConstants.wristKI,
    kD = Constants.WristConstants.wristKD,
    kIz = Constants.WristConstants.wristKIz,
    kFF = Constants.WristConstants.wristKFF, 
    kMinOutput = Constants.WristConstants.wristKMinOutput,
    kMaxOutput = Constants.WristConstants.wristKMaxOutput,
    minVel = Constants.WristConstants.wristMinVel,
    maxVel = Constants.WristConstants.wristMaxVel,
    maxAcc = Constants.WristConstants.wristMaxAcc,
    allowedErr = Constants.WristConstants.wristAllowedErr;
  
  

  public Wrist() {

  
    // This method will be called once per scheduler run
    wristMotor = new CANSparkMax(Constants.WristConstants.wristMotorID, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();
    wristMotor.setSmartCurrentLimit(30);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristMotor.setSoftLimit(SoftLimitDirection.kForward, 8);
    wristMotor.setSoftLimit(SoftLimitDirection.kReverse, -54);

    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // initialze PID controller and encoder objects
    wristPIDController = wristMotor.getPIDController();
    wristEncoder = wristMotor.getEncoder();
    reverseLimitSwitch = new DigitalInput(2);
    
    // set PID coefficients
    wristPIDController.setP(kP);
    wristPIDController.setI(kI);
    wristPIDController.setD(kD);
    wristPIDController.setIZone(kIz);
    wristPIDController.setFF(kFF);
    wristPIDController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    wristPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    wristPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    wristPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    wristPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    

  }

  public void setPosition(double targetPosition){
    wristPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  public void manualWrist(double speed){
    wristMotor.set(speed);
  }

  public double getOutputCurrent() {
    return wristMotor.getOutputCurrent();
  }

  public double getPosition() {
    return wristEncoder.getPosition();
  }

  public void resetEncoder(){
    wristEncoder.setPosition(8.3);
  }

  public void setEncoder(double position){
    wristEncoder.setPosition(position);
  }

  public boolean getLimitSwitch(){
    return !reverseLimitSwitch.get();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Current", getOutputCurrent());
    SmartDashboard.putNumber("Wrist Position", getPosition());
    SmartDashboard.putBoolean("Wrist Reverse Limit Switch", getLimitSwitch());
  }

}
