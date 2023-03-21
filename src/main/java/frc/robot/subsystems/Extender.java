// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ExtenderConstants;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Extender extends SubsystemBase {

  private final CANSparkMax extenderMotor;

  private SparkMaxPIDController extenderPIDController;
  private RelativeEncoder extenderEncoder;
  private DigitalInput reverseLimitSwitch;
  //private final TimeOfFlight extenderDistanceSensor;
  private final double forwardLimit, reverseLimit;
  
  double kP = Constants.ExtenderConstants.extenderKP,
    kI = Constants.ExtenderConstants.extenderKI,
    kD = Constants.ExtenderConstants.extenderKD,
    kIz = Constants.ExtenderConstants.extenderKIz,
    kFF = Constants.ExtenderConstants.extenderKFF, 
    kMinOutput = Constants.ExtenderConstants.extenderKMinOutput,
    kMaxOutput = Constants.ExtenderConstants.extenderKMaxOutput,
    minVel = Constants.ExtenderConstants.extenderMinVel,
    maxVel = Constants.ExtenderConstants.extenderMaxVel,
    maxAcc = Constants.ExtenderConstants.extenderMaxAcc,
    allowedErr = Constants.ExtenderConstants.extenderAllowedErr;

  
  /** Creates a new Elevator. */
  public Extender() {
    extenderMotor = new CANSparkMax(ExtenderConstants.extenderMotorID, MotorType.kBrushless);

    //extenderDistanceSensor = new TimeOfFlight(ExtenderConstants.extenderDistanceSensorID);  

    extenderMotor.restoreFactoryDefaults();
    extenderMotor.setSmartCurrentLimit(80);
    extenderMotor.setIdleMode(IdleMode.kBrake);
    reverseLimit = Units.inchesToMeters(0)*ExtenderConstants.KExtenderMetersToNeoRotationsFactor;
    forwardLimit = Units.inchesToMeters(20)*ExtenderConstants.KExtenderMetersToNeoRotationsFactor;

    extenderMotor.setSoftLimit(SoftLimitDirection.kForward,((float)forwardLimit));
    extenderMotor.setSoftLimit(SoftLimitDirection.kReverse, ((float)reverseLimit));

    extenderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    extenderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // initialze PID controller and encoder objects
    extenderPIDController = extenderMotor.getPIDController();
    extenderEncoder = extenderMotor.getEncoder();
    reverseLimitSwitch = new DigitalInput(0);

    // set PID coefficients
    extenderPIDController.setP(kP);
    extenderPIDController.setI(kI);
    extenderPIDController.setD(kD);
    extenderPIDController.setIZone(kIz);
    extenderPIDController.setFF(kFF);
    extenderPIDController.setOutputRange(kMinOutput, kMaxOutput);

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
    extenderPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    extenderPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    extenderPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    extenderPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

   // seedEncoder();
  }

  public void setPosition(double targetPosition){
    extenderPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  public void manualExtender(double speed){
    extenderMotor.set(speed);
  }

  public double getOutputCurrent() {
    return extenderMotor.getOutputCurrent();
  }

  public double getPosition() {
    return Units.metersToInches(extenderEncoder.getPosition()/ExtenderConstants.KExtenderMetersToNeoRotationsFactor);
  }

  public void resetEncoder(){
    extenderMotor.getEncoder().setPosition(0);
  }

  public boolean getLimitSwitch(){
    return !reverseLimitSwitch.get();
  }

  //public double getDistanceSensor(){
  //  return Units.metersToInches((extenderDistanceSensor.getRange()*.001)-.0127);
  //}

  //public void seedEncoder(){
  //  extenderEncoder.setPosition(Units.inchesToMeters(getDistanceSensor()*ExtenderConstants.KExtenderMetersToNeoRotationsFactor));
  //}



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extender Current", getOutputCurrent());
    SmartDashboard.putNumber("Extender Position", getPosition());
    //SmartDashboard.putNumber("Extender Distance Sensor Position", getDistanceSensor());
    SmartDashboard.putBoolean("Extend Reverse Limit Switch", getLimitSwitch());

    if(getLimitSwitch()){
      resetEncoder();
    }
  
  }

    /*
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Elevator Set P Gain", 0);
    double i = SmartDashboard.getNumber("Elevator Set I Gain", 0);
    double d = SmartDashboard.getNumber("Elevator Set D Gain", 0);
    double iz = SmartDashboard.getNumber("Elevator Set I Zone", 0);
    double ff = SmartDashboard.getNumber("Elevator Set Feed Forward", 0);
    double max = SmartDashboard.getNumber("Elevator Set Max Output", 0);
    double min = SmartDashboard.getNumber("Elevator Set Min Output", 0);
    double maxV = SmartDashboard.getNumber("Elevator Set Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Elevator Set Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Elevator Set Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Elevator Set Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { elevatorPIDController.setP(p); kP = p; }
    if((i != kI)) { elevatorPIDController.setI(i); kI = i; }
    if((d != kD)) { elevatorPIDController.setD(d); kD = d; }
    if((iz != kIz)) { elevatorPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { elevatorPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      elevatorPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { elevatorPIDController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { elevatorPIDController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { elevatorPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { elevatorPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    
    double setPoint = SmartDashboard.getNumber("Set Position", 0);
    
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
    /*
    elevatorPIDController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Output", elevatorMotor.getAppliedOutput());
    */
  
  
  }