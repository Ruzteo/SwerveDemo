package frc.robot;
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

    public void Update(){
        if(Constants.SwerveConstants.isSim){

        double currentAngleRad = angleSim.getAngularPositionRad();
        Rotation2d currentAngle = new Rotation2d(currentAngleRad);

        desiredState.optimize(currentAngle); 
        double desiredAngleRad = desiredState.angle.getRadians();
        anglePID.setGoal(desiredAngleRad);

        double voltagePID = anglePID.calculate(currentAngleRad);
        double voltageFF = angleFF.calculate(anglePID.getGoal().velocity); 
        double voltage = voltageFF + voltagePID; 
        voltage = MathUtil.clamp(voltage,-12, 12);
    
        angleSim.setInputVoltage(voltage);
        angleSim.update(Constants.SwerveConstants.kDt);

        }
        else{
            //TODO 
        } 
    }

    public void setDesiredState(SwerveModuleState state){
        desiredState = state; 
    }

    public SwerveModuleState getDesiredState(){
        return desiredState; 
    }


}
