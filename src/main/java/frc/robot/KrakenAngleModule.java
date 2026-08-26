package frc.robot;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class KrakenAngleModule {
    private int ID; 
    private int encoderID; 
    private DCMotorSim angleSim; 
    private TalonFX angle; 
    private SimpleMotorFeedforward angleFF; 
    private ProfiledPIDController anglePID; 
    private SwerveModuleState desiredState;
    private AbsoluteEncoder angleCoder; 

    public KrakenAngleModule(int ID, int encoderID){
        this.ID = ID; 
        this.encoderID = encoderID; 

        angleFF = new SimpleMotorFeedforward(Constants.SwerveConstants.kAngleFF_S, Constants.SwerveConstants.kAngleFF_V);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.SwerveConstants.kAngleMaxVelocity, Constants.SwerveConstants.kAngleMaxAcceleration);

        anglePID = new ProfiledPIDController(
            Constants.SwerveConstants.kAnglePID_P, Constants.SwerveConstants.kAnglePID_I, Constants.SwerveConstants.kAnglePID_D, constraints);
        
        if(Constants.SwerveConstants.isSim){
            angleSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, 1), 
            DCMotor.getKrakenX60(1));
       } 
       else{
        angle = new TalonFX(ID); 
       }
    }

    public void setAngle(double desiredAngle){
        if(Constants.SwerveConstants.isSim){

        Double currentAngle = angleSim.getAngularPosition().in(Degrees);
        anglePID.setGoal(desiredAngle);

        double voltagePID = anglePID.calculate(currentAngle);
        double voltageFF = angleFF.calculate(anglePID.getGoal().position); 
        double voltage = voltageFF + voltagePID; 
        voltage = MathUtil.clamp(voltage,-12, 12);
    
        angleSim.setInputVoltage(voltage);
        angleSim.update(Constants.SwerveConstants.kDt);

        }
        else{
            //TODO 
        } 
    }
    
    public Double getAngle(){
        if(Constants.SwerveConstants.isSim){
            return angleSim.getAngularPosition().in(Degrees);
        }
        else{
            return 0d;
            //TODO
        }
    }
}
