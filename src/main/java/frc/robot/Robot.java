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
    drivebase.reset();
  }

  @Override
  public void teleopPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    double txValue = getAprilTagValues(table, 1);
    SmartDashboard.putNumber("LimelightX", txValue);
    
    double x = xBox.getRawAxis(0)*0.4;
    double y = xBox.getRawAxis(1)*0.4;
    double z = xBox.getRawAxis(4)*0.4;


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

    if (turnToAng != -1){
      double curAng = drivebase.getNavX();
      if (Math.abs(curAng - turnToAng) < 0.5){
        turnToAng = -1;
      }else{
        double clockwise = (turnToAng - curAng + 360) % 360;
        double counterCwise = (curAng - turnToAng + 360) % 360;
        if (clockwise > counterCwise){
          z = -counterCwise / 180;
        }else{
          z = clockwise / 180;
        }
      }
    }

    if (xBox.getRawButtonPressed(2)){
      drivebase.reset();
    }


    drivebase.liveMove(y , x , z);
    drivebase.getEncoders();



  }

  private double getAprilTagValues(NetworkTable table, int id) {
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry tg = table.getEntry("camerapose_targetspace");
    
    //Reading values 
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double val = tid.getDouble(0.0);
    double[] cpose = tg.getDoubleArray(new double[]{0, 0, 0, 0, 0, 0, 0});
    if (val==id) {
      SmartDashboard.putNumber("LimelightX",x);
      SmartDashboard.putNumber("LimelightY",y);
      SmartDashboard.putNumber("targetZ",cpose[2]);
      SmartDashboard.putNumber("targetY",cpose[1]);
      SmartDashboard.putNumber("targetX",cpose[0]);
      return x;
      }
      else {
      SmartDashboard.putNumber("LimelightVal",val);
      return (-1.0);
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
