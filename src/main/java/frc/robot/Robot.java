// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  SwervBase drivebase = new SwervBase();
  Joystick xBox = new Joystick(0);
  Shooter johnathan = new Shooter(14,15,16);
  DigitalInput limitSwitch = new DigitalInput(0);
  VictorSPX climber = new VictorSPX(13);
  private int lastPOV = -1;
  private int turnToAng = -1;
  private boolean isAuto = false;
  private double[] redTags = {1, 2, 3, 4, 5, 6, 7, 8};
  private double[] blueTags = {9, 10, 11, 12, 13, 14, 15, 16};
  public double[] aprilTags;
  public boolean isShooting = false;
  public boolean isIntaking = false;
  public int targetRPM = 5800;


  double x;
  double y;
  double z;
  double m;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    NetworkTable tableFMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
    NetworkTableEntry isRedEntry = tableFMS.getEntry("IsRedAlliance");
    boolean isRed = isRedEntry.getBoolean(true);
    climber.setNeutralMode(NeutralMode.Brake);

    if (isRed) {
      aprilTags = redTags;
    }
    else {
      aprilTags = blueTags;
    }
    drivebase.reset();
  }

  @Override
  public void teleopPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double closestId = getClosest(table);
    double txValue = getAprilTagValueX(table, (int)closestId);
    double tzValue = getAprilTagValueZ(table, (int)closestId);

    SmartDashboard.putNumber("LimelightX", txValue);
    SmartDashboard.putNumber("LimelightZ", tzValue);
    SmartDashboard.putNumber("April tags", aprilTags[0]);

    SmartDashboard.putBoolean("Climb Switch", limitSwitch.get());
    
    x = xBox.getRawAxis(0);
    y = xBox.getRawAxis(1);
    z = xBox.getRawAxis(4);

    if (Math.abs(x) < 0.2){
      x = 0;
    }

    if (Math.abs(y) < 0.2){
      y = 0;
    }

    if (Math.abs(z) < 0.2){
      z = 0;
    }else{
      turnToAng = -1;
    }

    int currentPOV = xBox.getPOV();
    if (lastPOV == -1 && currentPOV != -1) {
      turnToAng = currentPOV;
    }
    lastPOV = currentPOV;


    if (xBox.getRawButtonPressed(1)){
      isAuto = !isAuto;
    }

    if(isAuto){
      tzValue = tzValue - 1;
      if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
        isAuto = false;
      }else{
      turnToAng = 0;
      if (txValue > 1) txValue = 1;
      if (txValue < -1) txValue = -1;
      if (tzValue > 1) tzValue = 1;
      if (tzValue < -1) tzValue = -1;
      x = -txValue;
      y = -tzValue;
      }
      
    }

    if (turnToAng != -1){
      double curAng = drivebase.getNavX();
      if (Math.abs(curAng - turnToAng) < 0.2){
        turnToAng = -1;
      }else{
        double clockwise = (turnToAng - curAng + 360) % 360;
        double counterCwise = (curAng - turnToAng + 360) % 360;
        if (clockwise > counterCwise){
          z = - Math.max(counterCwise / 180, 0.03);
        }else{
          z = Math.max(clockwise / 180, 0.03);
        }
      }
    }

    if (xBox.getRawButtonPressed(7)){
      drivebase.reset();
    }
    if (xBox.getRawButtonPressed(8)){
      drivebase.xToggle();
    }

    if (xBox.getRawButton(4)){
      johnathan.intake();
    }else{
      if (johnathan.getActiveShooter()){
        if (xBox.getRawButton(3)){
          isShooting = true;
        }
        if(isShooting){
          isShooting = johnathan.shoot();
        }else{
          johnathan.stop();
        }
    }else{
      if (xBox.getRawButton(3)){
        johnathan.shoot();
      }else{
        johnathan.stop();
      }
    }
      
    }

    if (xBox.getRawButtonPressed(2)){
      isIntaking = false;
      isShooting = false;
      johnathan.switchMode();
    }

      if (xBox.getRawButton(6) && limitSwitch.get()){
        climber.set(ControlMode.PercentOutput,-1);
      }else{
      if (xBox.getRawButton(5)){
        climber.set(ControlMode.PercentOutput,0.8);
      }else{
        climber.set(ControlMode.PercentOutput,0);
      }
    }

    johnathan.displayDiagnostics();
    m = 1 - (xBox.getRawAxis(3) * 0.8);
    drivebase.liveMove(y * m, x * m, z * m);
    drivebase.getEncoders();

  }

  private double getAprilTagValueX(NetworkTable table, int id) {
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry tg = table.getEntry("camerapose_targetspace");
    
    //Reading values 
    double val = tid.getDouble(0.0);
    double[] cpose = tg.getDoubleArray(new double[]{4688, 4688, 4688, 4688, 4688, 4688, 4688});
    if (val==id) {
      SmartDashboard.putNumber("targetX",cpose[0]);
      return cpose[0];
      }
      else {
      SmartDashboard.putNumber("LimelightVal",val);
      return (4688.2024);
      }
      
  }

  private double getAprilTagValueZ(NetworkTable table, int id) {
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry tg = table.getEntry("camerapose_targetspace");
    
    //Reading values 
    double val = tid.getDouble(0.0);
    double[] cpose = tg.getDoubleArray(new double[]{4688, 4688, 4688, 4688, 4688, 4688, 4688});
    if (val==id) {
      SmartDashboard.putNumber("targetX", Math.abs(cpose[2]));
      return Math.abs(cpose[2]);
      }
      else {
      SmartDashboard.putNumber("LimelightVal",val);
      return (4688.2024);
      }
      
  }

  private double getClosest(NetworkTable table) {
    NetworkTableEntry tid = table.getEntry("tid");
    double id = tid.getDouble(0.0);
    return id;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
