package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter {
    private static final int SHOOTSTART = 300;
    private static final int TOPCLAWSENSOR = 0;
    private static final int INTAKESENSOR = 1;

    private boolean intake = false;
    private boolean revup = false;
    private boolean isShooter = false;

    private final CANSparkMax flyWheel;
    private final CANSparkMax ShootMotor;
    private final CANSparkMax handMotor;
    private final RelativeEncoder encoder;
    private  SparkPIDController pidController;
    private int targetRPM = SHOOTSTART;

    private final AnalogInput handSensor;
    private final AnalogInput shootSensor;

    public Shooter(int flyID, int shootID, int handID) {
        // Defining Motors
        flyWheel = new CANSparkMax(flyID, MotorType.kBrushless);
        ShootMotor = new CANSparkMax(shootID, MotorType.kBrushless);
        handMotor = new CANSparkMax(handID, MotorType.kBrushless);

        handSensor = new AnalogInput(TOPCLAWSENSOR);
        shootSensor = new AnalogInput(INTAKESENSOR);

        // Defining Encoder
        encoder = flyWheel.getEncoder();

        pidController = flyWheel.getPIDController();

        // PID Coefficients and parameters
        pidController.setP(0.1);
        pidController.setI(1e-4);
        pidController.setD(1);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(-1, 1);
    }

    public boolean inTake(){
        if (isShooter){
            double voltage = shootSensor.getVoltage();
            if (voltage < 4) {
                ShootMotor.set(0);
                flyWheel.set(0);
                return false;
            }
            else {
                ShootMotor.set(-1);
                flyWheel.set(-1);
                return true;
            }
        }else{
            double voltage = handSensor.getVoltage();
            if (voltage < 4) {
                handMotor.set(0);
                return false;
            }
            else {
                handMotor.set(-1);
                return true;
            }
        }
    }

    public boolean Shoot(){
        if (isShooter){
            double rpm = encoder.getVelocity();
            pidController.setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
            if (Math.abs(targetRPM - rpm) <= 50) {
                ShootMotor.set(1);
                double voltage = shootSensor.getVoltage();
                if (voltage < 4) {
                    ShootMotor.set(0);
                    flyWheel.set(0);
                    return false;
                }else{
                    return true;
                }
            }
        }else{
            double voltage = handSensor.getVoltage();
            if (voltage < 4) {
                handMotor.set(0);
                return false;
            }else {
                handMotor.set(1);
                return true;
            }
        }
        return true;
    }

    public void Switch(){
        isShooter = !isShooter;
        handMotor.set(0);
        ShootMotor.set(0);
        flyWheel.set(0);
    }

}
