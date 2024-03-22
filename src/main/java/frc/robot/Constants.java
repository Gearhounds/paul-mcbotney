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
    public static final int DPAD_UP = 0;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_LEFT = 270;

    public static final double SHOOT_TIMEOUT = 1; // sec
    public static final double DEFAULT_SHOOTER_SPEED = .65; // percent
    public static final double SHOOTER_SPEED_UP = -.4;
    public static final double SHOOTER_SPEED_DOWN = .1;
    public static final double JOYSTICK_DEADZONE = 0.05;

    public static final double[] RED_SPEAKER_IDS = {3, 4};
    public static final double[] BLUE_SPEAKER_IDS = {7, 8};
    
    public static final int[] COLOR_OFF = {0, 0, 0};
    public static final int[] COLOR_PURPLE = {255, 0, 255};
    public static final int[] COLOR_RED = {255, 0, 0};
    public static final int[] COLOR_AMBER = {255, 150, 0};
    public static final int[] COLOR_GREEN = {0, 255, 0};
    public static final int[] COLOR_BLUE = {0, 0, 255};

    // add more constants as needed
}