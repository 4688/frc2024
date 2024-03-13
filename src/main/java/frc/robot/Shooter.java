package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private static final int SHOOT_START_RPM = 5800;
    private static final int TOP_CLAW_SENSOR_PORT = 0;
    private static final int INTAKE_SENSOR_PORT = 1;

    private boolean startTimer = true;
    private boolean isShooterActive = true;
    private int fireRPM = SHOOT_START_RPM;

    private CANSparkMax flywheelMotor;
    private CANSparkMax shootMotor;
    private CANSparkMax handMotor;
    private RelativeEncoder flywheelEncoder;

    private Timer timer = new Timer();
    private AnalogInput handSensor = new AnalogInput(TOP_CLAW_SENSOR_PORT);
    private AnalogInput shootSensor = new AnalogInput(INTAKE_SENSOR_PORT);

    public Shooter(int flywheelID, int shootID, int handID) {
        // Initialize motors
        flywheelMotor = new CANSparkMax(flywheelID, MotorType.kBrushless);
        shootMotor = new CANSparkMax(shootID, MotorType.kBrushless);
        handMotor = new CANSparkMax(handID, MotorType.kBrushless);

        // Encoder and PID Controller for the flywheel
        flywheelEncoder = flywheelMotor.getEncoder();
    }

    public boolean intake() {
        if (isShooterActive) {
            startTimer();
            double voltage = shootSensor.getVoltage();
            if (voltage > 4) {
                shootMotor.set(0);
                flywheelMotor.set(0);
                return false;
            } else {
                shootMotor.set(-1);
                flywheelMotor.set(-1);
                return true;
            }
        } else {
            double voltage = handSensor.getVoltage();
            if (voltage < 4) {
                handMotor.set(0);
                return false;
            } else {
                handMotor.set(0.5);
                return true;
            }
        }
    }

    public boolean shoot() {
        if (isShooterActive) {
            double rpm = flywheelEncoder.getVelocity();

            if (rpm >= fireRPM) {
                shootMotor.set(1);
                if (shootSensor.getVoltage() < 4) {
                    startTimer();
                }
            }
            
            if (timer.get() > 1.0) {
                shootMotor.set(0);
                flywheelMotor.set(0);
                stopTimer();
                return false;
            }else{
                flywheelMotor.set(1);
                return true;
            }

        } else {
                handMotor.set(-1);
                return true;
        }
    }

    public void switchMode() {
        isShooterActive = !isShooterActive;
        handMotor.set(0);
        shootMotor.set(0);
        flywheelMotor.set(0);
        stopTimer();
    }

    public void stop() {
        handMotor.set(0);
        shootMotor.set(0);
        flywheelMotor.set(0);
        stopTimer();
    }

    public boolean getActiveShooter(){
        return isShooterActive;
    }

    private void startTimer() {
        if (startTimer) {
            timer.reset();
            timer.start();
            startTimer = false;
        }
    }

    private void stopTimer() {
        timer.stop();
        timer.reset();
        startTimer = true;
    }

    public void setfireRPM(int target) {
        this.fireRPM = target;
    }

    public void displayDiagnostics() {
        SmartDashboard.putNumber("Hand Sensor Voltage", handSensor.getVoltage());
        SmartDashboard.putNumber("Intake Sensor Voltage", shootSensor.getVoltage());
        SmartDashboard.putNumber("Target RPM", fireRPM);
        SmartDashboard.putNumber("Current RPM", flywheelEncoder.getVelocity());
    }
}
