package frc.util;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PIDFGains {
    private final double _kP, _kI, _kD, _kF, _tolerance, _iZone, _maxVelocity, _maxAcceleration;
  
    public PIDFGains(double kP, double kI, double kD, double kF, double tolerance, double iZone) {
      this._kP = kP;
      this._kI = kI;
      this._kD = kD;
      this._kF = kF;
      this._tolerance = tolerance;
      this._iZone = iZone;
      this._maxAcceleration = 0;
      this._maxVelocity = 0;
    }
  
    public PIDFGains(double kP, double kI, double kD) {
      this._kP = kP;
      this._kI = kI;
      this._kD = kD;
      this._kF = 0;
      this._tolerance = 0;
      this._iZone = 0;
      this._maxAcceleration = 0;
      this._maxVelocity = 0;
    }

    public PIDFGains(double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
      this._kP = kP;
      this._kI = kI;
      this._kD = kD;
      this._kF = 0;
      this._tolerance = 0;
      this._iZone = 0;
      this._maxVelocity = maxVelocity;
      this._maxAcceleration = maxAcceleration;
    }

    public PIDFGains(double kP, double kI, double kD, double kF, double tolerance, double iZone, double maxVelocity, double maxAcceleration) {
      this._kP = kP;
      this._kI = kI;
      this._kD = kD;
      this._kF = kF;
      this._tolerance = tolerance;
      this._iZone = iZone;
      this._maxVelocity = maxVelocity;
      this._maxAcceleration = maxAcceleration;
    }

    public double getMaxVelocity() {
      return _maxVelocity;
    }

    public double getMaxAcceleration() {
      return _maxAcceleration;
    }

    public double getIZone() {
        return _iZone;
    }

    public double getP() {
      return this._kP;
    }
  
    public double getI() {
      return this._kI;
    }
  
    public double getD() {
      return this._kD;
    }
  
    public double getF() {
      return this._kF;
    }
  
    public double getTolerance() {
      return this._tolerance;
    }

    public PIDConstants createPIDConstants(){
      return new PIDConstants(_kP, _kI, _kD, _iZone);
    }

    public PIDController createPIDController() {
      var controller = new PIDController(_kP, _kI, _kD);
      controller.setTolerance(_tolerance);
      controller.setIZone(_iZone);
      return controller;
    }

    public ProfiledPIDController createProfiledPIDController() {
      return new ProfiledPIDController(_kP, _kI, _kD, new TrapezoidProfile.Constraints(_maxVelocity , _maxAcceleration));
    }
  }