package frc.robot.subsystems.Shooter_Intake;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.CAN;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

public class ShooterIntakeIONewClass implements ShooterIntakeIO{
    MarinersController intakeMotor;
    MarinersController onPivotShooterMotor;
    MarinersController offPivotShooterMotor;

    public ShooterIntakeIONewClass(){
        intakeMotor = new MarinersSparkBase("intake motor",
                ControllerLocation.MOTOR,
                ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_ID,
                true,
                MotorType.SPARK_FLEX,
                ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_PID);

        intakeMotor.getMeasurements().setGearRatio(3);

        
        onPivotShooterMotor = new MarinersSparkBase("on pivot shooter motor",
                ControllerLocation.MOTOR,
                ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_MOTOR_ID,
                true,
                MotorType.SPARK_FLEX,
                ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_PID);

        offPivotShooterMotor = new MarinersSparkBase("off pivot shooter motor",
                ControllerLocation.MOTOR,
                ShooterIntakeConstants.IO_CONSTNATS.OFF_PIVOT_SHOOTER_MOTOR_ID,
                true,
                MotorType.SPARK_FLEX,
                ShooterIntakeConstants.IO_CONSTNATS.OFF_PIVOT_SHOOTER_PID);
    }
    public void stopIntakeMotor (){
        this.intakeMotor.stopMotor();
    }
    public void StopMotorOnPivot (){
        this.onPivotShooterMotor.stopMotor();
    }
    public void StopMotorOffPivot (){
        this.offPivotShooterMotor.stopMotor();
    }
    public void setTargetShooterMotorOnPivotRPM(double RPM){
        this.intakeMotor.setReference(RPM/60, ControlMode.Velocity);
    }
    public void setTargetShooterMotorOffPivotRPM(double RPM){
        this.offPivotShooterMotor.setReference(RPM/60, ControlMode.Velocity);
    }
    public void setTargetShooterMotorOffPivotDutyCycle(double dutyCycle){
        this.offPivotShooterMotor.setDutyCycle(dutyCycle);
    }
    public void setTargetShooterMotorOnPivotDutyCycle(double dutyCycle){
        this.onPivotShooterMotor.setDutyCycle(dutyCycle);
    }
    public void setIntakeTargetPosition(double position){
        this.intakeMotor.setReference(position, ControlMode.Position);
    }
    @Override
    public void setTargetIntakeMotorDutyCycle(double dutyCycle) {
        this.intakeMotor.setDutyCycle(dutyCycle);
    }
    @Override
    public void update(ShooterIntakeInputsAutoLogged inputs) {
    
        inputs.intakeMotorPosition = this.intakeMotor.getPosition();
        inputs.intakeMotorRPM = this.intakeMotor.getVelocity();

        //on pivot shooter motor
        inputs.onPivotShooterMotorRPM = this.onPivotShooterMotor.getVelocity();

        //off pivot shooter motor
        inputs.offPivotShooterMotorRPM = this.offPivotShooterMotor.getVelocity();
    }
    
}
