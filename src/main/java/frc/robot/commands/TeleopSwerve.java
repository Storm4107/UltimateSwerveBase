package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.rev.RevSwerve;
import frc.robot.subsystems.swerve.rev.RevSwerveConfig;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private RevSwerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier dampen;
    private DoubleSupplier speedDial;

    public TeleopSwerve(RevSwerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen, DoubleSupplier speedDial) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.dampen = dampen;
        this.speedDial = speedDial;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(RevSwerveConfig.maxSpeed), 
            rotationVal * RevSwerveConfig.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}