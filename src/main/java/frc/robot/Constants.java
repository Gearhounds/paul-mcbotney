package frc.robot;

public final class Constants {

    // constructor
    private Constants() {
        // prevents instantiation as an object
    }

    // constants are static so they can be referenced off the class, final so they are constant, and public so we can
    // access them outside of the class. Convention is to use uppercase snake_case when naming constants. We want to
    // use constants in place of all magic numbers (apart from 0), for example in things like motor speeds or, soft limits, etc.
    // By using this file, we can change them all at once if we need to tune the robot in anyway.

    public static final int TEAM_NUMBER = 5674;
    public static final double SHOOT_TIMEOUT = 2; // sec

    // add more constants as needed
}