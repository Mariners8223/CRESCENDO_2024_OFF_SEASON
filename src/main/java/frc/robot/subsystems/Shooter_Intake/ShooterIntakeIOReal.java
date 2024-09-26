package frc.robot.subsystems.Shooter_Intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import frc.util.PIDFGains;
public class ShooterIntakeIOReal implements ShooterIntakeIO {
    CANSparkFlex intakeMotor;
    CANSparkFlex onPivotShooterMotor;
    CANSparkFlex offPivotShooterMotor;

    public ShooterIntakeIOReal(){
        intakeMotor = configureIntakeMotor();
        onPivotShooterMotor = configureOnPivotShooterMotor();
        offPivotShooterMotor = configureOffPivotShooterMotor();
    }

    private CANSparkFlex configureIntakeMotor(){
       return configureMotor(ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_ID,
       ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_INVERTED,
       ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_PID);
    }

    public CANSparkFlex configureOnPivotShooterMotor(){
     return configureMotor(ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_MOTOR_ID,
       ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_MOTOR_INVERTED,
       ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_PID);
    }

    public CANSparkFlex configureOffPivotShooterMotor(){
 return configureMotor(ShooterIntakeConstants.IO_CONSTNATS.OFF_PIVOT_SHOOTER_MOTOR_ID,
       ShooterIntakeConstants.IO_CONSTNATS.OFF_PIVOT_SHOOTER_MOTOR_INVERTED,
       ShooterIntakeConstants.IO_CONSTNATS.OFF_PIVOT_SHOOTER_PID);
    }

    public void setTargetIntakeMotorRPM(double speedRPM){
        this.intakeMotor.getPIDController().setReference(speedRPM, ControlType.kVelocity);

    }
    public void setTargetShooterMotorOffPivotRPM(double speedRPM){
        this.offPivotShooterMotor.getPIDController().setReference(speedRPM, ControlType.kVelocity);

    }
    public void setTargetShooterMotorOnPivotRPM(double speedRPM){
        this.onPivotShooterMotor.getPIDController().setReference(speedRPM, ControlType.kVelocity);

    }
    public void StopMotorOffPivot(){
        this.offPivotShooterMotor.stopMotor();

    }
    public void StopMotorOnPivot(){
        this.onPivotShooterMotor.stopMotor();

    }
    public void stopIntakeMotor() {
        this.intakeMotor.stopMotor();

    }
    public void setIntakeTargetPosition(double rotation){
        rotation *= ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_GEAR_RATIO;
        this.intakeMotor.getPIDController().setReference(rotation, ControlType.kPosition);

    }
    public void update(ShooterIntakeInputsAutoLogged inputs){
        
        // update shooterIntake variables
        //intake motor
        inputs.intakeMotorCurrent = this.intakeMotor.getOutputCurrent();
        inputs.intakeMotorPosition = this.intakeMotor.getEncoder().getPosition() / ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_GEAR_RATIO;
        inputs.intakeMotorRPM = this.intakeMotor.getEncoder().getVelocity();

        //on pivot shooter motor
        inputs.onPivotShooterMotorCurrent = this.onPivotShooterMotor.getOutputCurrent();
        inputs.onPivotShooterMotorRPM = this.onPivotShooterMotor.getEncoder().getVelocity();

        //off pivot shooter motor
        inputs.offPivotShooterMotorCurrent = this.offPivotShooterMotor.getOutputCurrent();
        inputs.offPivotShooterMotorRPM = this.offPivotShooterMotor.getEncoder().getVelocity();
    }


    public void setTargetIntakeMotorDutyCycle(double dutyCycle) {
        // Ensure the duty cycle is within the valid range (0.0 to 1.0 for forward power, -1.0 to 0.0 for reverse power)
        dutyCycle = MathUtil.clamp(dutyCycle, -1, 1);

        // Set the motor power directly based on the duty cycle (percent output)
        intakeMotor.set(dutyCycle);  // Assuming intakeMotor is a motor controller (e.g., PWMVictorSPX or TalonSRX)
    }
     public void setTargetShooterMotorOnPivotDutyCycle(double dutyCycle) {
        // Ensure the duty cycle is within the valid range (0.0 to 1.0 for forward power, -1.0 to 0.0 for reverse power)
        dutyCycle = MathUtil.clamp(dutyCycle, -1, 1);

        // Set the motor power directly based on the duty cycle (percent output)
        onPivotShooterMotor.set(dutyCycle);  // Assuming intakeMotor is a motor controller (e.g., PWMVictorSPX or TalonSRX)
    }
     public void setTargetShooterMotorOffPivotDutyCycle(double dutyCycle) {
        // Ensure the duty cycle is within the valid range (0.0 to 1.0 for forward power, -1.0 to 0.0 for reverse power)
        dutyCycle = MathUtil.clamp(dutyCycle, -1, 1);

        // Set the motor power directly based on the duty cycle (percent output)
        offPivotShooterMotor.set(dutyCycle);  // Assuming intakeMotor is a motor controller (e.g., PWMVictorSPX or TalonSRX)
    }

    
    private CANSparkFlex configureMotor(int motorID, boolean isInverted, PIDFGains gains){
        CANSparkFlex motor = new CANSparkFlex(motorID, CANSparkFlex.MotorType.kBrushless);

        motor.setInverted(isInverted);

        motor.getPIDController().setP(gains.getP());
        motor.getPIDController().setI(gains.getI());
        motor.getPIDController().setD(gains.getD());
        motor.getPIDController().setFF(gains.getF());
        return motor;
    }
}
