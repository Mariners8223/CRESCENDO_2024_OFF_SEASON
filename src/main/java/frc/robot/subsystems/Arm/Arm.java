// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;


public class Arm extends SubsystemBase {
    /**
     * Creates a new arm.
     */
    private final ArmIO io;
    private final ArmInputsAutoLogged inputs;
    private boolean isCalibrated;
    private ArmPosition currentPos;
    private Mechanism2d armMechanism;
    private MechanismRoot2d root;
    private MechanismLigament2d alpha2d;
    private MechanismLigament2d beta2d;

    public Arm() {
        io = new ArmIOReal();
        inputs = new ArmInputsAutoLogged();
        isCalibrated = false;
        currentPos = ArmPosition.HOME_POSITION;
        
        armMechanism = new Mechanism2d(1.3, 1, new Color8Bit(Color.kBlack));
        root = armMechanism.getRoot("Arm Root", 0.3, 0);
        alpha2d = root.append(new MechanismLigament2d("Alpha Arm", 0.43, 0, 6, new Color8Bit(Color.kBlue)));
        beta2d = alpha2d.append(new MechanismLigament2d("Beta Arm", 0.36, 0, 4, new Color8Bit(Color.kLightBlue)));
    }

    public void moveAlpha(double alphaTarget) {

        double target = Math.max(-ArmConstants.AlphaConstants.REVERSE_SOFT_LIMIT,
                Math.min(alphaTarget, ArmConstants.AlphaConstants.FORWARD_SOFT_LIMIT));

        io.setAlphaTargetRotation(target);

        Logger.recordOutput("Arm/Alpha Target", target);
    }

    public void moveBeta(double betaTarget) {

        double target = Math.max(ArmConstants.BetaConstants.REVERSE_SOFT_LIMIT,
                Math.min(betaTarget, ArmConstants.BetaConstants.FORWARD_SOFT_LIMIT));

        io.setBetaTargetRotation(target);

        Logger.recordOutput("Arm/Beta Target", target);
    }

    public void moveBetaDutyCycle(double speed) {
        speed = MathUtil.clamp(speed,
                ArmConstants.BetaConstants.MIN_OUTPUT_RANGE, ArmConstants.BetaConstants.MAX_OUTPUT_RANGE);

        io.moveBetaDutyCycle(speed);
    }

    public void enableBetaSoftLimits(){
        io.enableBetaSoftLimits();
    }

    public void stopAlpha() {
        io.stopAlpha();
    }

    public void stopBeta() {
        io.stopBeta();
    }

    public boolean getLimitSwitch() {
        return inputs.betaLimitSwitch;
    }

    public void resetBetaEncoder() {
        io.resetBetaEncoder();
    }

    public double getAlphaPosition() {
        return inputs.motorAlphaPosition;
    }

    public double getBetaPosition() {
        return inputs.motorBetaPosition;
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public void setArmCalibrated() {
        isCalibrated = true;
    }

    public ArmPosition getCurrentPos() {
        return currentPos;
    }

    public void setArmPositionUnknown() {
        currentPos = ArmPosition.UNKNOWN;
    }

    public double getBetaMotorCurrent(){ return inputs.betaMotorCurrent;}

    @Override
    public void periodic() {
        io.update(inputs);
        Logger.processInputs("Arm", inputs);

        double alpha = getAlphaPosition();
        double beta = getBetaPosition();

        currentPos = findArmPosition(alpha, beta);

        alpha2d.setAngle(Units.rotationsToDegrees(inputs.motorAlphaPosition));
        beta2d.setAngle(Units.rotationsToDegrees(inputs.motorBetaPosition));
        Logger.recordOutput("Arm/Current Pose", currentPos);
        Logger.recordOutput("Arm/Mechanism2D", armMechanism);

        double alphaRadians = -1*Units.rotationsToRadians(alpha);
        double betaRadians = -1*Units.rotationsToRadians(beta);

        Pose3d alphaArmPose3d = new Pose3d(ArmConstants.ALPHA_DISTANCE_FROM_CENTER_CUSTOM_ASSETS, new Rotation3d(0, alphaRadians, 0));

        double x = Math.cos(-alphaRadians) * ArmConstants.DISTANCE_BETWEEN_PIVOTS_METERS + alphaArmPose3d.getX();
        double y = -0.125;
        double z = Math.sin(-alphaRadians) * ArmConstants.DISTANCE_BETWEEN_PIVOTS_METERS + alphaArmPose3d.getZ();

        Pose3d betaArmPose3d = new Pose3d(new Translation3d(x, y, z), new Rotation3d(0, Math.PI + (alphaRadians + betaRadians), 0));
        Logger.recordOutput("Arm/Alpha Position", alphaArmPose3d);
        Logger.recordOutput("Arm/Beta Position", betaArmPose3d);

        String currentCommand = getCurrentCommand() != null ? getCurrentCommand().getName() : "None";
        Logger.recordOutput("Arm/Current Command", currentCommand);
    }

    private ArmPosition findArmPosition(double alpha, double beta) {
        ArmPosition[] positions = ArmPosition.values();

        for (ArmPosition armPos : positions) {
            if (Math.abs(alpha - armPos.getAlpha()) < ArmConstants.ARM_POSITION_TOLERANCE &&
                    Math.abs(beta - armPos.getBeta()) < ArmConstants.ARM_POSITION_TOLERANCE) return armPos;
        }

        return ArmPosition.UNKNOWN;
    }
}
