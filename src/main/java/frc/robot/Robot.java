// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  SwervBase drivebase;
  EveryBot kitBot;
  Limelight limey;
  Myah driveController;
  
  private boolean isAuto = false;
  private int curAuto = 0;
  private int curAutoStep = 0;


  /*DA AUTO MASTER PLAN (TOP SECRET NO ONE CAN KNOW) 
   * 1 - when A is pressed, log the current ID, SKEW, X and Z, if ID = 0, END
   * 2 - Set Acentric to the current Apriltag scew and drive the X vector only till x = 0
   * 3 - Turn the robot to the saved skew + Cur Position
   * 4 - Recapture Apriltag to update desired X and Z if ID is correct while driving the desired X and Z distance in Acentric to updated scew. 
   * 5 - Once found, stop. (Maybe do something else);
   * 
   * AutoChecker -> WHen A is pressed, if ID is good for alliance, set Autohandler to current ID 
   * AutoHandler -> Always running, if variable is not set to 0, call handler for specific ID
   * SpecificHandler -> Keep track of autostep, when currentID is reset, so should the step, run the code based on the step.
  */

  double x;
  double y;
  double autoX;
  double autoZ;
  double z;
  double m;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    drivebase = new SwervBase();
    kitBot = new EveryBot();
    limey = new Limelight();
    driveController = new Myah();
    drivebase.resetNavx();
    drivebase.setToGlideMode();
    limey.checkField();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    drivebase.resetNavx();
    limey.checkField();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    limey.checkField();
  }

  

  @Override
  public void teleopPeriodic() {
   if (driveController.getAButton()){
      isAuto = !isAuto;
  }

    if(isAuto){
      ampAuto(0);
    }else{
      teleopDrive();
    }

    kitBot.displayDiagnostics();
    drivebase.getEncoders();
    
  }

  @Override
  public void disabledInit() {
    drivebase.setToCoastMode();
  }

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

  public void teleopDrive(){
    x = driveController.getX();
    y = driveController.getY();
    z = driveController.getZ(drivebase.getNavX());

    if (driveController.getStartButton()){
      drivebase.resetNavx();
    }

    if (driveController.getSelectButton()){
      drivebase.xToggle();
    }

    kitBot.handleNoteIn(driveController.getYButton());
    kitBot.handleNoteOut(driveController.getXButton());
    
    if (driveController.getBButton()){
      kitBot.switchMode();
    }

    kitBot.handleClimb(driveController.getClimbButton(), driveController.getRaiseButton(), driveController.getSafteyButton());

    m =  driveController.getBrake();
    drivebase.fieldCentric(x * m, y * m, z * m);
  }


  public void startAuto(boolean Autobutton){
    limey.updateLimelight();
    if(limey.canSee()){
      SmartDashboard.putString("AUTO LOG", "STARTING AUTO FOR ID: " + limey.getID());
      curAuto = limey.getID();
      isAuto = true;
    }
  }

  public void handleAuto(){
    if(isAuto){
      
    }
  }

  public void ampAuto(int turnAng){
    if (curAutoStep == 0){
      if (limey.canSee()){
        autoX = limey.getLimelightX();
        autoZ = limey.getLimelightZ();
        curAutoStep = 1;
        drivebase.resetDistance();
      }else{
        isAuto = false;
      }
    }else if(curAutoStep == 1){
      if(LineDriveTo(autoX, (360 + turnAng - drivebase.getNavX()) % 360)){
        curAutoStep = 2;
        driveController.turnTo(turnAng);
      }
    }else if(curAutoStep == 2){
      double turnZ = driveController.getAutoZ(drivebase.getNavX());
      drivebase.robotCentric(0, 0, turnZ);
      if(turnZ > -0.01 && turnZ < 0.01){
        curAutoStep = 3;
        drivebase.resetDistance();
      }
    }else if(curAutoStep == 3){
      limey.updateLimelight();
      if(limey.getID() == 4 || limey.getID() == 7){
        autoX = limey.getLimelightX();
        autoZ = limey.getLimelightZ();
      }
      if(limelightDriveTo(autoX, autoZ)){
        curAutoStep = 0;
        curAuto = 0;
        isAuto = false;
      }
    }
  }

  public boolean limelightDriveTo(double x, double z){
    double startDist = drivebase.calculateMagnitude(x, y);
    if (startDist < 0.05){
      drivebase.robotCentric(0, 0, 0);
      return true;
    }
    double distance = startDist - drivebase.getDistance();
    if(Math.abs(distance) < 0.05){
      drivebase.robotCentric(0, 0, 0);
      return true;
    }
    drivebase.robotCentric((x/startDist)*distance, (z/startDist)*distance, 0);
    return false;
  }

  public boolean LineDriveTo(double x, double a){
    double startDist = x;
    if (startDist < 0.05){
      return true;
    }
    double distance = startDist - drivebase.getDistance();
    if(Math.abs(distance) < 0.05){
      drivebase.robotCentric(0, 0, 0);
      return true;
    }
    drivebase.aCentric((x/startDist)*distance, 0, 0, a);
    return false;
  }






  
}
