package org.firstinspires.ftc.teamcode.api;

public class State {

    public static class Buttons {
        public boolean a = false;
        public boolean b = false;
        public boolean x = false;
        public boolean y = false;

        public void update(boolean aNew, boolean bNew, boolean xNew, boolean yNew){
            a = aNew;
            b = bNew;
            x = xNew;
            y = yNew;
        }
    }

    public static class Dpad {
        public boolean dpad_up = false;
        public boolean dpad_down = false;
        public boolean dpad_right = false;
        public boolean dpad_left = false;

        public void update(boolean dpad_upNew, boolean dpad_downNew, boolean dpad_rightNew, boolean dpad_leftNew){
            dpad_up = dpad_upNew;
            dpad_down = dpad_downNew;
            dpad_right = dpad_rightNew;
            dpad_left = dpad_leftNew;
        }
    }

    public static class Bumpers {
        public boolean right_bumper = false;
        public boolean left_bumper = false;

        public void update(boolean bumper_rightNew, boolean bumper_leftNew) {
            right_bumper = bumper_rightNew;
            left_bumper = bumper_leftNew;
        }
    }

}
