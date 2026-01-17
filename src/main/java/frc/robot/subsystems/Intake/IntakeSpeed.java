package frc.robot.subsystems.Intake;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

public class IntakeSpeed implements Interpolatable<IntakeSpeed> {
    double motorSpeed;

    public IntakeSpeed(double motorSpeed){
        this.motorSpeed = motorSpeed;
    }

    private IntakeSpeed diff(IntakeSpeed o){
        return new IntakeSpeed(
            this.motorSpeed - o.motorSpeed
        );
    }

    private IntakeSpeed sum(IntakeSpeed o){
        return new IntakeSpeed(
            this.motorSpeed + o.motorSpeed
        );
    }

    private IntakeSpeed product(double scalar){
        return new IntakeSpeed(
            this.motorSpeed * scalar
        );
    }

    @Override
    public IntakeSpeed interpolate(IntakeSpeed endValue, double t) {
        IntakeSpeed delta = this.diff(endValue);
        IntakeSpeed interpolated = this.sum(delta.product(t));
        return interpolated;
    }
    
    static Interpolator<IntakeSpeed> getInterpolator() {
        return IntakeSpeed :: interpolate;
    }
}

