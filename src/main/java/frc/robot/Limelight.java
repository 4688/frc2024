package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight {
    private NetworkTable table;
    private int closestID;
    private double x;
    private double z;
    private NetworkTableEntry tid;
    private NetworkTableEntry tg;
    private double[] cpose;
    private boolean isRed;
    private  ArrayList<Integer> redNums = new ArrayList<>(Arrays.asList(3,4,9,10,5));
    private  ArrayList<Integer> blueNums = new ArrayList<>(Arrays.asList(7,8,1,2,6));

    public void updateLimelight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tid = table.getEntry("tid");
        tg = table.getEntry("camerapose_targetspace");
        closestID = (int) tid.getInteger(0);
        cpose = tg.getDoubleArray(new double[]{4688, 4688, -4688, 4688, 4688, 4688, 4688});
        x = cpose[0];
        z = -cpose[2];

        SmartDashboard.putNumber("Closest ID",closestID);
        SmartDashboard.putNumber("targetX",x);
        SmartDashboard.putNumber("targetZ",z);

    }

    public int getID(){
        return closestID;
    }

    public double getLimelightX(){
        return x;
    }

    public double getLimelightZ(){
        return z;
    }

    public boolean canSee(){
        if(closestID == 0){
            return false;
        }
        if(isRed){
            return redNums.contains(closestID);
        }else{
            return blueNums.contains(closestID);
        }
    }

    public boolean areWeRed() {
        return isRed;
    }

    public void checkField() {
        NetworkTable tableFMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
        NetworkTableEntry isRedEntry = tableFMS.getEntry("IsRedAlliance");
        isRed = isRedEntry.getBoolean(true);
    }

}
