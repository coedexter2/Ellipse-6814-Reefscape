package frc.robot;

public enum ReefAlignment
{
    LEFT,
    MIDDLE,
    RIGHT;
    
    public ReefAlignment opposite()
    {
        if(this == LEFT)
        {
            return RIGHT;
        }
        else if (this == RIGHT)
        {
            return LEFT;
        }
        else
        {
            return MIDDLE;
        }
    }
}