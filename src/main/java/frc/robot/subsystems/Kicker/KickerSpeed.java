package frc.robot.subsystems.Kicker;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

public class KickerSpeed implements Interpolatable<KickerSpeed> {
    double motorSpeed;

    public KickerSpeed(double motorSpeed){
        this.motorSpeed = motorSpeed;
    }

    private KickerSpeed diff(KickerSpeed o){
        return new KickerSpeed(
            this.motorSpeed - o.motorSpeed
        );
    }

    private KickerSpeed sum(KickerSpeed o){
        return new KickerSpeed(
            this.motorSpeed + o.motorSpeed
        );
    }

    private KickerSpeed product(double scalar){
        return new KickerSpeed(
            this.motorSpeed * scalar
        );
    }

    @Override
    public KickerSpeed interpolate(KickerSpeed endValue, double t) {
        KickerSpeed delta = this.diff(endValue);
        KickerSpeed interpolated = this.sum(delta.product(t));
        return interpolated;
    }
    
    static Interpolator<KickerSpeed> getInterpolator() {
        return KickerSpeed :: interpolate;
    }
}
