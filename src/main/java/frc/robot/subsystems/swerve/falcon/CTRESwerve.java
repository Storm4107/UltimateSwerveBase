package frc.robot.subsystems.swerve.falcon;

import frc.lib.math.GeometryUtils;
import frc.robot.constants.CTRESwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CTRESwerve extends SubsystemBase {
    
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private SwerveDrivePoseEstimator swervePose;
    public Pose2d robotPose;

    double previousT;
    double offT;
    Timer timer = new Timer();

    private Rotation2d targetHeading;

    public CTRESwerve() {
        gyro = new Pigeon2(CTRESwerveConstants.Swerve.pigeonID, "CANivore"); //"rio" (default), or the name of your CANivore
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, CTRESwerveConstants.Swerve.Mod0.constants),
                new SwerveModule(1, CTRESwerveConstants.Swerve.Mod1.constants),
                new SwerveModule(2, CTRESwerveConstants.Swerve.Mod2.constants),
                new SwerveModule(3, CTRESwerveConstants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.*/
        Timer.delay(1.0);
        resetModulesToAbsolute();
        
    }

   
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02; 
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));   
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;  
    }
    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed){
        //Determine time interval
        double currentT = timer.get();
        double dt = currentT - previousT;
        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);
        if (vr > 0.01 || vr < -0.01){
            offT = currentT;
            setTargetHeading(getYaw());
            return desiredSpeed;
        }
        if (currentT - offT < 0.5){
            setTargetHeading(getYaw());
            return desiredSpeed;
        }
        //Determine target and current heading
        setTargetHeading( getTargetHeading().plus(new Rotation2d(vr * dt)) );
        Rotation2d currentHeading = getYaw();
        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = getTargetHeading().minus(currentHeading);
        if (Math.abs(deltaHeading.getDegrees()) < 0.05){
            return desiredSpeed;
        }
        double correctedVr = deltaHeading.getRadians() / dt * 0.05;
        previousT = currentT;

        return new ChassisSpeeds(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, correctedVr);
    }
    public Rotation2d getTargetHeading(){ 
        return targetHeading; 
    }
    public void setTargetHeading(Rotation2d targetHeading) { 
        this.targetHeading = targetHeading; 
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {        
        ChassisSpeeds desiredChassisSpeeds =
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYaw())
            : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        desiredChassisSpeeds = correctHeading(desiredChassisSpeeds);
        
        SwerveModuleState[] swerveModuleStates = CTRESwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, CTRESwerveConfig.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void stopDrive() {
        drive(new Translation2d(0, 0), 0, false, true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, CTRESwerveConfig.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }
    public Pose2d getPose(){
        return robotPose;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (CTRESwerveConfig.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }


    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }
    
    public double gyroTemp(){
        return gyro.getTemp();
    }

    @Override
    public void periodic() {
        robotPose = swervePose.update(getYaw(), getModulePositions());        
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("CTRE Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("CTRE Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("CTRE Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}