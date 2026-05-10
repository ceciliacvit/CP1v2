function [angle_vel,headingErrorInt,headingErrorPrev,headingError] = look_at(heading,desired_heading,Kp_h,Ki_h,Kd_h,headingErrorInt,headingErrorPrev)
    arguments
        heading;
        desired_heading;
        Kp_h;
        Ki_h;
        Kd_h;
        headingErrorInt = 0;
        headingErrorPrev = 0;
    end

    headingError    = atan2(sin(desired_heading - heading), cos(desired_heading - heading));
    headingErrorInt = headingErrorInt + headingError * dt;
    headingErrorDer = (headingError - headingErrorPrev) / dt;
    headingErrorPrev = headingError;

    angularVelocity = Kp_h * headingError ...
                + Ki_h * headingErrorInt ...
                + Kd_h * headingErrorDer;

    angle_vel = clip(angularVelocity, -4.0, 4.0);

end