package frc.robot.subsystems.Shooter_Intake;

import java.net.CacheRequest;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.util.PIDFGains;

public class ShooterIntakeIOReal implements ShooterIntakeIO {
    CANSparkFlex intakemotor;
    CANSparkFlex onPivotShooterMotor;
    CANSparkFlex offPivotShooterMotor;
    DigitalInput beamBreak;

    public ShooterIntakeIOReal(){
        intakemotor = configureIntakeMotor();
    }

    public CANSparkFlex configureIntakeMotor(){
        CANSparkFlex motor = new CANSparkFlex(ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_ID, MotorType.kBrushless);

        motor.setInverted(ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_INVERTED);

        motor.getPIDController().setP(ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_PID.getP());
        motor.getPIDController().setI(ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_PID.getI());
        motor.getPIDController().setD(ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_PID.getD());
        motor.getPIDController().setFF(ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_PID.getF());
    }

    public CANSparkFlex configureonPivotShooterMotor(){
        CANSparkFlex motor = new CANSparkFlex(ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

        motor.setInverted(ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_MOTOR_INVERTED);

        motor.getPIDController().setP(ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_PID.getP());
        motor.getPIDController().setI(ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_PID.getI());
        motor.getPIDController().setD(ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_PID.getD());
        motor.getPIDController().setFF(ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_PID.getF());
    }

    public void setTargetIntakeMotorRPM(double speedRPM){

    }
    public void setTargetShooterMotorOffPivotRPM(double speedRPM){

    }
    public void setTargetShooterMotorOnPivotRPM(double speedRPM){

    }
    public void StopMotorOffPivot(double speedRPM){

    }
    public void StopMotorOnPivot(double speedRPM){

    }
    public void setIntakeTargetPosition(double speedRPM){

    }
    public void update(ShooterIntakeInputsAutoLogged inputs){

    }
    private CANSparkFlex configureMotor(int motorID, boolean isInverted, PIDFGains){
        CANSparkFlex motor = new CANSparkFlex(motorID, CANSparkFlex.MotorType.kBrushless);

        Motor.setInverted(isInverted);

        motor.getPIDController().setP(gains.getP());
        motor.getPIDController().setI(gains.getI());
        motor.getPIDController().setD(gains.getD());
        motor.getPIDController().setFF(gains.getF());
        
    }
}
