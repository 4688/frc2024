// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private int lastPOV = -1;
  private int turnToAng = -1;
  private boolean isAuto = false;

  double x;
  double y;
  double z;

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
    int[] redTags = {1, 2, 3, 4, 5, 6, 7, 8};
    int[] blueTags = {9, 10, 11, 12, 13, 14, 15, 16};
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    drivebase.reset();
  }

  @Override
  public void teleopPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double txValue = getAprilTagValueX(table, 1);
    double tzValue = getAprilTagValueZ(table, 1);

    SmartDashboard.putNumber("LimelightX", txValue);
    SmartDashboard.putNumber("LimelightZ", tzValue);
    
    x = xBox.getRawAxis(0)*0.4;
    y = xBox.getRawAxis(1)*0.4;
    z = xBox.getRawAxis(4)*0.4;

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
      if (Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02){
        isAuto = false;
      }else{
      turnToAng = 0;
      x = -txValue;
      y = -tzValue;
      }
      
    }

    if (turnToAng != -1){
      double curAng = drivebase.getNavX();
      if (Math.abs(curAng - turnToAng) < 0.5){
        turnToAng = -1;
      }else{
        double clockwise = (turnToAng - curAng + 360) % 360;
        double counterCwise = (curAng - turnToAng + 360) % 360;
        if (clockwise > counterCwise){
          z = - Math.max(counterCwise / 180, 0.06);
        }else{
          z = Math.max(clockwise / 180, 0.06);
        }
      }
    }

    if (xBox.getRawButtonPressed(2)){
      drivebase.reset();
    }
    if (xBox.getRawButtonPressed(3)){
      drivebase.xToggle();
    }

    double m = xBox.getRawAxis(2);
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
