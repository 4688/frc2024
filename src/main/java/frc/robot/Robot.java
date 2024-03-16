// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  // Hardware components
  private SwervBase drivebase = new SwervBase(); // Controls the robot's movement
  private Joystick xBox = new Joystick(0); // Input device for controlling the robot
  private Shooter johnathan = new Shooter(14, 15, 16); // The robot's shooting mechanism
  private DigitalInput limitSwitch = new DigitalInput(0); // A limit switch for detecting physical contact
  private VictorSPX climber = new VictorSPX(13); // Motor controller for the climbing mechanism
  private Spark lights = new Spark(0); // Controller for lights

  // State variables
  private boolean isAuto = false; // Tracks whether the robot is in autonomous mode
  private boolean isShooting = false; // Tracks whether the robot is currently shooting
  private boolean isRed = true; // Tracks whether the robot is on the Red alliance
  private int contID = 0; // A control identifier for managing state in autonomous or teleop modes
  private int lastPOV = -1; // Tracks the last POV value (direction of the D-pad) from the joystick
  private int turnToAng = -1; // Desired angle to turn to during operations

  // Positional and movement variables
  private double saveZ = 0; // Temporary storage for Z-axis value
  private double saveX = 0; // Temporary storage for X-axis value
  private double x = 0; // General purpose variable for X-axis manipulation
  private double y = 0; // General purpose variable for Y-axis manipulation
  private double z = 0; // General purpose variable for Z-axis manipulation
  private double m = 0; // A variable for another measurement or magnitude (the purpose is unclear without context)
  
  private double txValue = 0;
  private double tzValue = 0;
  private double curAng = 0;
  private int closestId = 0;

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
    Optional<Alliance> ally = DriverStation.getAlliance();
    isRed = (ally.get() == Alliance.Red);
    drivebase.reset();
    lights.set(-0.75);
  }

  @Override
  public void teleopPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    closestId = (int) getClosest(table);
    txValue = getAprilTagValueX(table, closestId);
    tzValue = getAprilTagValueZ(table, closestId);
    curAng = drivebase.getNavX();

    SmartDashboard.putNumber("LimelightX", txValue);
    SmartDashboard.putNumber("LimelightZ", tzValue);

    SmartDashboard.putBoolean("IS AUTO", isAuto);
    SmartDashboard.putNumber("LIMELIGHT ID", closestId);
    SmartDashboard.putNumber("AUTO STEP", contID);
    SmartDashboard.putBoolean("ALLIANCE", isRed);

    SmartDashboard.putBoolean("Climb Switch", limitSwitch.get());
    johnathan.displayDiagnostics();
    drivebase.getEncoders();
    


    x = applyDeadzone(xBox.getRawAxis(0), 0.07);
    y = applyDeadzone(xBox.getRawAxis(1), 0.07);
    z = applyDeadzone(xBox.getRawAxis(4), 0.07);

    if (z != 0) {
        turnToAng = -1;
    }

    int currentPOV = xBox.getPOV();
    if (lastPOV == -1 && currentPOV != -1) {
        turnToAng = currentPOV;
    }
    lastPOV = currentPOV;


    if (turnToAng != -1) {
        adjustTurnAngle(curAng);
    }


    if (xBox.getRawButtonPressed(1)){
      isAuto = !isAuto;
      drivebase.setTurnOffset(0);
    }

    if (xBox.getRawButtonPressed(7)){
      drivebase.reset();
    }
    if (xBox.getRawButtonPressed(8)){
      drivebase.xToggle();
    }

    if (xBox.getRawButton(4)) {
      johnathan.intake();
    } else {
      if (xBox.getRawButton(3)) {
          if (johnathan.getActiveShooter()) {
              isShooting = true;
          } else {
              johnathan.shoot();
          }
      }
      if (isShooting) {
          isShooting = johnathan.shoot();
          if (!isShooting){
            johnathan.stop();
          }
      }
    }

    if (xBox.getRawButtonPressed(2)){
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

    if (isAuto) {
      handleAutoMode();
    } else {
      drivebase.setTurnOffset(0);
    }

    m = 1 - (xBox.getRawAxis(3) * 0.8);
    drivebase.liveMove(y * m, x * m, z * m);
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



























  private double applyDeadzone(double value, double threshold) {
    return Math.abs(value) < threshold ? 0 : value;
  }

  private void adjustTurnAngle(double curAng) {
    double angleDifference = Math.abs(curAng - turnToAng);
    
    if (angleDifference < 0.2) {
        turnToAng = -1; // Reset turnToAng if close to target
    } else {
        calculateTurnDirection(curAng);
    }
  }

  private void calculateTurnDirection(double curAng) {
    double clockwiseDistance = (turnToAng - curAng + 360) % 360;
    double counterClockwiseDistance = (curAng - turnToAng + 360) % 360;

    if (clockwiseDistance < counterClockwiseDistance) {
        z = Math.max(clockwiseDistance / 180, 0.03);
    } else {
        z = -Math.max(counterClockwiseDistance / 180, 0.03);
    }
  }

  private void handleAutoMode() {
    if (isRed) {
        handleRedTeamLogic();
    } else {
        handleBlueTeamLogic();
    }
  }

  private void handleRedTeamLogic() {
    if(contID == 4){
          if(!johnathan.shoot()){
            isAuto = false;
            drivebase.setTurnOffset(0);
            contID = 0;
          }
        }
        else if(closestId == 4){
          double dist = Math.sqrt(Math.pow(txValue, 2) + Math.pow(tzValue, 2));
          double angle = Math.toDegrees(Math.atan2(tzValue, txValue)) - 180;
          if (angle < 0) angle += 360;
          if (txValue > 1) txValue = 1;
          if (txValue < -1) txValue = -1;
          if (tzValue > 1) tzValue = 1;
          if (tzValue < -1) tzValue = -1;
          drivebase.setTurnOffset(180);
          turnToAng = (int) angle;
          if (dist > 2.25){
            x = -txValue;
            y = -tzValue;
          }else if(dist < 1.2){
            x = txValue;
            y = tzValue;
          }else{
            turnToAng = (int) (angle + 26.5) % 360;
            contID = 4;
          }
        }

        else if(closestId == 3){
          double angle = Math.toDegrees(Math.atan2(tzValue, txValue - 0.616)) - 180;
          if (angle < 0) angle += 360;
          turnToAng = (int) angle % 360;
        }

        if(closestId == 9 || closestId == 10){
          if (johnathan.getActiveShooter()){
            txValue = txValue - 0.1;
          }else{
            txValue = txValue + 0.1;
          }
          
          if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
            contID = 9;
            drivebase.resetDistance();
          }else{
            turnToAng = 300;
            drivebase.setTurnOffset(300);
            if (txValue > 1) txValue = 1;
            if (txValue < -1) txValue = -1;
            if (tzValue > 1) tzValue = 1;
            if (tzValue < -1) tzValue = -1;
            x = -txValue;
            y = -tzValue;
            saveZ = txValue;
            saveX = tzValue;
          }
        }
        else if(contID == 9){
          double distToTarget = Math.sqrt(Math.pow(saveX, 2) + Math.pow(saveZ, 2)) - drivebase.getDistance();
          if(distToTarget < 0.01 && distToTarget > -0.01){
            contID = 0;
            drivebase.resetDistance();
          }else{
            double regy = saveZ /(Math.abs(saveX) + Math.abs(saveZ));
            double regx = saveX / (Math.abs(saveX) + Math.abs(saveZ));
            y = -regy * distToTarget;
            x = -regx * distToTarget;
          }
        }else if(closestId == 5){
          txValue = txValue + 0.1;
          if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
            contID = 5;
            drivebase.resetDistance();
          }else{
            turnToAng = 90;
            drivebase.setTurnOffset(90);
            if (txValue > 1) txValue = 1;
            if (txValue < -1) txValue = -1;
            if (tzValue > 1) tzValue = 1;
            if (tzValue < -1) tzValue = -1;
            x = -txValue;
            y = -tzValue;
            saveZ = txValue;
            saveX = tzValue;
          }
        }
        else if(contID == 5){
          double distToTarget = Math.sqrt(Math.pow(saveX, 2) + Math.pow(saveZ, 2)) - drivebase.getDistance();
          if(distToTarget < 0.01 && distToTarget > -0.01){
            contID = 0;
            drivebase.resetDistance();
          }else{
            double regy = saveZ /(Math.abs(saveX) + Math.abs(saveZ));
            double regx = saveX / (Math.abs(saveX) + Math.abs(saveZ));
            y = -regy * distToTarget;
            x = -regx * distToTarget;
          }
        }else{
          isAuto = false;
          drivebase.setTurnOffset(0);
        }
      }




  private void handleBlueTeamLogic() {
    if(contID == 7){
          if(!johnathan.shoot()){
            SmartDashboard.putNumber("PING", 4688);
            isAuto = false;
            drivebase.setTurnOffset(0);
            contID = 0;
          }
        }
        else if(closestId == 7){
          double dist = Math.sqrt(Math.pow(txValue, 2) + Math.pow(tzValue, 2));
          double angle = Math.toDegrees(Math.atan2(tzValue, txValue))-90;
          if (angle < 0) angle += 360;
          if (txValue > 1) txValue = 1;
          if (txValue < -1) txValue = -1;
          if (tzValue > 1) tzValue = 1;
          if (tzValue < -1) tzValue = -1;
          turnToAng = (int) angle;
          SmartDashboard.putNumber("dist", dist);
          SmartDashboard.putNumber("angle", angle);
          drivebase.setTurnOffset(180);
          if (dist > 2.25){
            x = -txValue;
            y = -tzValue;
          }else if(dist < 1.2){
            x = txValue;
            y = tzValue;
          }else{
            turnToAng = (int) (angle + 26.5) % 360;
            contID = 7;
          }
        }

        else if(closestId == 8){
          double angle = Math.toDegrees(Math.atan2(tzValue, txValue + 0.616)) - 180;
          if (angle < 0) angle += 360;
          turnToAng = (int) angle % 360;
        }

        if(closestId == 1 || closestId == 2){
          if (johnathan.getActiveShooter()){
            txValue = txValue - 0.1;
          }else{
            txValue = txValue + 0.1;
          }
          
          if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
            contID = 1;
            drivebase.resetDistance();
          }else{
            turnToAng = 60;
            drivebase.setTurnOffset(60);
            if (txValue > 1) txValue = 1;
            if (txValue < -1) txValue = -1;
            if (tzValue > 1) tzValue = 1;
            if (tzValue < -1) tzValue = -1;
            x = -txValue;
            y = -tzValue;
            saveZ = txValue;
            saveX = tzValue;
          }
        }
        else if(contID == 1){
          double distToTarget = Math.sqrt(Math.pow(saveX, 2) + Math.pow(saveZ, 2)) - drivebase.getDistance();
          if(distToTarget < 0.01 && distToTarget > -0.01){
            contID = 0;
            drivebase.resetDistance();
          }else{
            double regy = saveZ /(Math.abs(saveX) + Math.abs(saveZ));
            double regx = saveX / (Math.abs(saveX) + Math.abs(saveZ));
            y = -regy * distToTarget;
            x = -regx * distToTarget;
          }
        }else if(closestId == 6){
          txValue = txValue + 0.1;
          if ((Math.abs(txValue) < 0.02 && Math.abs(tzValue) < 0.02) || tzValue > 200){
            contID = 6;
            drivebase.resetDistance();
          }else{
            turnToAng = 270;
            drivebase.setTurnOffset(270);
            if (txValue > 1) txValue = 1;
            if (txValue < -1) txValue = -1;
            if (tzValue > 1) tzValue = 1;
            if (tzValue < -1) tzValue = -1;
            x = -txValue;
            y = -tzValue;
            saveZ = txValue;
            saveX = tzValue;
          }
        }
        else if(contID == 6){
          double distToTarget = Math.sqrt(Math.pow(saveX, 2) + Math.pow(saveZ, 2)) - drivebase.getDistance();
          if(distToTarget < 0.01 && distToTarget > -0.01){
            contID = 0;
            drivebase.resetDistance();
          }else{
            double regy = saveZ /(Math.abs(saveX) + Math.abs(saveZ));
            double regx = saveX / (Math.abs(saveX) + Math.abs(saveZ));
            y = -regy * distToTarget;
            x = -regx * distToTarget;
          }
        }else{
          //isAuto = false;
          drivebase.setTurnOffset(0);
        }
  }
}
