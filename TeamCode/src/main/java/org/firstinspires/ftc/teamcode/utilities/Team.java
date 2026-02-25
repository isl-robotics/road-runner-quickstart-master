package org.firstinspires.ftc.teamcode.utilities;

public enum Team {
    RED,BLUE;

    public static Team active;
    public static Team get(){ return active; }
    public static void set(Team team){ active = team; }
}
