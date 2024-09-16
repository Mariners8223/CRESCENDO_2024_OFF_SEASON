package frc.robot.subsystems.Shooter_Intake;

import java.net.CacheRequest;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.util.PIDFGains;

public class ShooterIntakeIOReal implements ShooterIntakeIO {
    CANSparkFlex intakemotor;
    CANSparkFlex onPivotShooterMotorCurrent;
    CANSparkFlex offPivotShooterMotorCurrent;
    DigitalInput beamBreak;

    public ShooterIntakeIOReal(){

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
