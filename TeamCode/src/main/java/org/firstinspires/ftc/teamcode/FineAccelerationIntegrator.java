package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Created by Andrew lu on 12/3/17.
 */

public class FineAccelerationIntegrator implements BNO055IMU.AccelerationIntegrator {

    BNO055IMU.Parameters parameters;
    Position position;
    Velocity velocity;
    Acceleration acceleration;
    //------------------------------------------------------------------------------------------
    // Construction
    //------------
    FineAccelerationIntegrator()
    {
        this.parameters = null;
        this.position = new Position();
        this.velocity = new Velocity();
        this.acceleration = null;
    }

    @Override
    public void initialize(@NonNull BNO055IMU.Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity) {
        this.parameters = parameters;
        this.position = initialPosition != null ? initialPosition : this.position;
        this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
        this.acceleration = null;
    }

    public Position getPosition() { return this.position; }
    public Velocity getVelocity() { return this.velocity; }
    public Acceleration getAcceleration() { return this.acceleration; }

    @Override
    public void update(Acceleration linearAcceleration) {

        // We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0)
        {
            // We can only integrate if we have a previous acceleration to baseline from
            if (acceleration != null)
            {
                Acceleration accelPrev    = acceleration;
                Velocity     velocityPrev = velocity;

                acceleration = linearAcceleration;

                if (accelPrev.acquisitionTime != 0)
                {
                    Velocity deltaVelocity = meanIntegrate(acceleration, accelPrev);
                    velocity = plus(velocity, deltaVelocity);
                }

                if (velocityPrev.acquisitionTime != 0)
                {
                    Position deltaPosition = meanIntegrate(velocity, velocityPrev);
                    position = plus(position, deltaPosition);
                }

                if (parameters != null && parameters.loggingEnabled)
                {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime)*1e-9, acceleration, velocity, position);
                }
            }
            else
                acceleration = linearAcceleration;
        }
    }
}
