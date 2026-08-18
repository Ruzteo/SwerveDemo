package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class KrakenDriveModule {
    private int ID; 
    private DCMotorSim driveSim; 
    private TalonFX drive; 
    private SimpleMotorFeedforward driveFF; 
    private ProfiledPIDController drivePID; 
    private SwerveModuleState desiredState; 

    public KrakenDriveModule(int ID){
        this.ID = ID; 

        driveFF = new SimpleMotorFeedforward(Constants.SwerveConstants.kDriveFF_S, Constants.SwerveConstants.kAngleFF_V);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.SwerveConstants.kDriveMaxVelocity, Constants.SwerveConstants.kDriveMaxAcceleration);

        drivePID = new ProfiledPIDController(
            Constants.SwerveConstants.kDrivePID_P, Constants.SwerveConstants.kDrivePID_I, Constants.SwerveConstants.kDrivePID_D, constraints);
        
        if(Constants.SwerveConstants.isSim){
            driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, 1), 
            DCMotor.getKrakenX60(1));
       } 
       else{
        drive = new TalonFX(ID); 
       }
    }

    public void Update(){
        if(Constants.SwerveConstants.isSim){

        double currentVelocityRad = driveSim.getAngularVelocityRadPerSec();
        double currentVelocity = currentVelocityRad / Units.inchesToMeters(2);
        double desiredvelocity = desiredState.speedMetersPerSecond; 
        drivePID.setGoal(desiredvelocity);

        double voltagePID = drivePID.calculate(currentVelocity);
        double voltageFF = driveFF.calculate(drivePID.getGoal().velocity); 
        double voltage = voltageFF + voltagePID; 
        voltage = MathUtil.clamp(voltage,-12, 12);
    
        driveSim.setInputVoltage(voltage);
        driveSim.update(Constants.SwerveConstants.kDt);
        }
    }

    public void setDesiredState(SwerveModuleState state){
        desiredState = state; 
    }

    public SwerveModuleState getDesiredState(){
        return desiredState; 
    }


}
