package frc.robot;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EveryBot {
    private static final int SHOOT_RPM = 6100;
    private static final double SHOOT_OVERRIDE_TIME = 2.5;
    private static final double CLAW_DROP_TIME = 0.5;

    private static final int TOP_CLAW_SENSOR_PORT = 0;
    private static final int INTAKE_SENSOR_PORT = 1;
    private static final int FLYWHEEL_ID = 14;
    private static final int INTAKE_ID = 15;
    private static final int CLAW_WHEEL_ID = 16;
    private static final int CLIMB_LIMIT_SWITCH_PORT = 0;
    private static final int CLIMBER_PORT = 13;
    private static final int LIGHTS_PORT = 0;

    private boolean usingShooter = true;
    private int shooterStep = 0;
    private boolean isClimbing = false;

    private CANSparkMax flywheelMotor;
    private CANSparkMax intakeMotor;
    private CANSparkMax clawMotor;
    private RelativeEncoder flywheelEncoder;

    private AnalogInput clawSensor;
    private AnalogInput intakeSensor;

    private DigitalInput climbStopSwitch;
    private VictorSPX climber;

    private Spark lights;

    private final Timer timer;

    public EveryBot() {
        flywheelMotor = new CANSparkMax(FLYWHEEL_ID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(INTAKE_ID, MotorType.kBrushless);
        clawMotor = new CANSparkMax(CLAW_WHEEL_ID, MotorType.kBrushless);

        clawSensor = new AnalogInput(TOP_CLAW_SENSOR_PORT);
        intakeSensor = new AnalogInput(INTAKE_SENSOR_PORT);

        climbStopSwitch = new DigitalInput(CLIMB_LIMIT_SWITCH_PORT);
        climber = new VictorSPX(CLIMBER_PORT);
        lights = new Spark(LIGHTS_PORT);

        flywheelEncoder = flywheelMotor.getEncoder();
        timer = new Timer();
    }

    public void handleLights(boolean canSeeLights) {
        if (!climbStopSwitch.get()) {
            lights.set(-0.99); // IF WE ARE CLIMBED
        } else if (isClimbing) {
            lights.set(0.53); // IF WE ARE CLIMBING
        } else if (canSeeLights){
            lights.set(-0.83);
        }else if (usingShooter) {
            if (intakeSensor.getVoltage() > 4) {
                lights.set(0.19); // IF WE HAVE A PEICE IN SHOOTER
            } else {
                lights.set(0.67); // IF WE DO NOT HAVE A PEICE IN SHOOTER
            }
        } else {
            if (clawSensor.getVoltage() < 4) {
                lights.set(-0.01); // IF WE HAVE A PEICE IN CLAW
            } else {
                lights.set(0.87); // IF WE DO NOT HAVE A PEICE IN CLAW
            }
        }
    }

    public void handleNoteIn(boolean isPressed) {
        if (usingShooter) {
            shooterIntake(isPressed);
        } else {
            clawIntake(isPressed);
        }
    }

    public boolean shooterIntake(boolean isPressed) {
        if (isPressed) {
            double voltage = intakeSensor.getVoltage();
            if (voltage > 4) {
                intakeMotor.set(0);
                flywheelMotor.set(0);
                return false;
            } else {
                intakeMotor.set(-1);
                flywheelMotor.set(-1);
                return true;
            }
        } else {
            intakeMotor.set(0);
            flywheelMotor.set(0);
            return false;
        }
    }

    public boolean clawIntake(boolean isPressed) {
        if (isPressed) {
            double voltage = clawSensor.getVoltage();
            if (voltage < 4) {
                clawMotor.set(0);
                return false;
            } else {
                clawMotor.set(1);
                return true;
            }
        } else {
            clawMotor.set(0);
            return false;
        }
    }

    public boolean shoot() {
        if (shooterStep == 0) {
            timer.start();
            shooterStep = 1;
            flywheelMotor.set(1);
        } else if (shooterStep == 1) {
            double rpm = flywheelEncoder.getVelocity();
            if (rpm >= SHOOT_RPM || timer.get() > SHOOT_OVERRIDE_TIME) {
                intakeMotor.set(1);
                timer.reset();
                shooterStep = 2;
            }
        } else if (shooterStep == 2) {
            if (timer.get() > 0.5) {
                flywheelMotor.set(0);
                intakeMotor.set(0);
                shooterStep = 0;
                timer.reset();
                timer.stop();
                return false;
            }
        }
        return true;
    }

    public boolean clawDrop(boolean isPressed) {
        if (isPressed) {
            clawMotor.set(-1);
            return true;
        } else {
            clawMotor.set(0);
            return false;
        }

    }

    public boolean clawSeq() {
        if (shooterStep == 0) {
            timer.start();
            clawMotor.set(-1);
        } else if (timer.get() > CLAW_DROP_TIME) {
            clawMotor.set(0);
            timer.reset();
            timer.stop();
            return false;

        }
        return true;
    }

    public void handleNoteOut(boolean isPressed) {
        if (usingShooter) {
            if (isPressed || shooterStep > 0) {
                shoot();
            }else{
                flywheelMotor.set(0);
                intakeMotor.set(0);
            }
        } else {
            clawDrop(isPressed);
        }
    }

    public void switchMode() {
        usingShooter = !usingShooter;
        clawMotor.set(0);
        intakeMotor.set(0);
        flywheelMotor.set(0);
        timer.reset();
        timer.stop();
        shooterStep = 0;
    }

    public boolean isShooterSelected() {
        return usingShooter;
    }

    public void handleClimb(boolean climbButton, boolean raiseButton, boolean safteyButton) {
        if (safteyButton && raiseButton) {
            climber.set(ControlMode.PercentOutput, 0.8);
        } else if (climbStopSwitch.get() && climbButton) {
            climber.set(ControlMode.PercentOutput, -1);
        } else {
            climber.set(ControlMode.PercentOutput, 0);
        }
        isClimbing = climbButton;
    }

    public void displayDiagnostics() {
        SmartDashboard.putNumber("Hand Sensor Voltage", clawSensor.getVoltage());
        SmartDashboard.putNumber("Intake Sensor Voltage", intakeSensor.getVoltage());
        SmartDashboard.putNumber("Current RPM", flywheelEncoder.getVelocity());
        SmartDashboard.putBoolean("Climb Switch", climbStopSwitch.get());
    }
}
