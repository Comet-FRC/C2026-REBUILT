/*package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

public class ShooterSpeed implements Interpolatable<ShooterSpeed> {
    double motorSpeed;

    public ShooterSpeed(double motorSpeed){
        this.motorSpeed = motorSpeed;
    }

    private ShooterSpeed diff(ShooterSpeed o){
        return new ShooterSpeed(
            this.motorSpeed - o.motorSpeed
        );
    }

    private ShooterSpeed sum(ShooterSpeed o){
        return new ShooterSpeed(
            this.motorSpeed + o.motorSpeed
        );
    }

    private ShooterSpeed product(double scalar){
        return new ShooterSpeed(
            this.motorSpeed * scalar
        );
    }

    @Override
    public ShooterSpeed interpolate(ShooterSpeed endValue, double t) {
        ShooterSpeed delta = this.diff(endValue);
        ShooterSpeed interpolated = this.sum(delta.product(t));
        return interpolated;
    }

    static Interpolator<ShooterSpeed> getInterpolator() {
        return ShooterSpeed :: interpolate;
    }
}*/
