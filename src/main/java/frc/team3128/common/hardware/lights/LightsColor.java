package frc.team3128.common.hardware.lights;


/**
 * Class to represent a color that can be displayed on the robot lights.
 * @author Jamie
 *
 */
public class LightsColor
{
	// copied from java.awt.color, which doesn't exist in the robot JRE
    /**
     * The LightsColor white.  In the default sRGB space.
     */
    public final static LightsColor white     = new8Bit(255, 255, 255);

    /**
     * The LightsColor white.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor WHITE = white;

    /**
     * The LightsColor light gray.  In the default sRGB space.
     */
    public final static LightsColor lightGray = new8Bit(192, 192, 192);

    /**
     * The LightsColor light gray.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor LIGHT_GRAY = lightGray;

    /**
     * The LightsColor gray.  In the default sRGB space.
     */
    public final static LightsColor gray      = new8Bit(128, 128, 128);

    /**
     * The LightsColor gray.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor GRAY = gray;

    /**
     * The LightsColor dark gray.  In the default sRGB space.
     */
    public final static LightsColor darkGray  = new8Bit(64, 64, 64);

    /**
     * The LightsColor dark gray.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor DARK_GRAY = darkGray;

    /**
     * The LightsColor black.  In the default sRGB space.
     */
    public final static LightsColor black     = new8Bit(0, 0, 0);

    /**
     * The LightsColor black.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor BLACK = black;

    /**
     * The LightsColor red.  In the default sRGB space.
     */
    public final static LightsColor red       = new8Bit(255, 0, 0);

    /**
     * The LightsColor red.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor RED = red;

    /**
     * The LightsColor pink.  In the default sRGB space.
     */
    public final static LightsColor pink      = new8Bit(255, 175, 175);

    /**
     * The LightsColor pink.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor PINK = pink;

    /**
     * The LightsColor orange.  In the default sRGB space.
     */
    public final static LightsColor orange    = new8Bit(255, 200, 0);

    /**
     * The LightsColor orange.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor ORANGE = orange;

    /**
     * The LightsColor yellow.  In the default sRGB space.
     */
    public final static LightsColor yellow    = new8Bit(255, 255, 0);

    /**
     * The LightsColor yellow.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor YELLOW = yellow;

    /**
     * The LightsColor green.  In the default sRGB space.
     */
    public final static LightsColor green     = new8Bit(0, 255, 0);

    /**
     * The LightsColor green.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor GREEN = green;

    /**
     * The LightsColor magenta.  In the default sRGB space.
     */
    public final static LightsColor magenta   = new8Bit(255, 0, 255);

    /**
     * The LightsColor magenta.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor MAGENTA = magenta;

    /**
     * The LightsColor cyan.  In the default sRGB space.
     */
    public final static LightsColor cyan      = new8Bit(0, 255, 255);

    /**
     * The LightsColor cyan.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor CYAN = cyan;

    /**
     * The LightsColor blue.  In the default sRGB space.
     */
    public final static LightsColor blue      = new8Bit(0, 0, 255);

    /**
     * The LightsColor blue.  In the default sRGB space.
     * @since 1.4
     */
    public final static LightsColor BLUE = blue;

    //--------------------------------------------
	//actual 11-bit RGB values.
	private int r, g, b;
	
	/**
	 * Highest acceptable value for a single rgb channel
	 */
	public static final int HIGHEST_COLOR_VALUE = 0x7ff;
	
	private LightsColor(int r, int g, int b)
	{
		this.r = r;
		this.g = g;
		this.b = b;
	}
	
	/**
	 * Create a new color from 11-bit values (up to 0x7ff).
	 * @param rChannel
	 * @param gChannel
	 * @param bChannel
	 * @return
	 */
	public static LightsColor new11Bit(int rChannel, int gChannel, int bChannel)
	{
		return new LightsColor(rChannel, gChannel, bChannel);
	}
	
	/**
	 * Create a new color from 8-bit values (up to 0xff).
	 * @param rChannel
	 * @param gChannel
	 * @param bChannel
	 * @return
	 */
	public static LightsColor new8Bit(int rChannel, int gChannel, int bChannel)
	{
		return new LightsColor(rChannel*8, gChannel*8, bChannel*8);
	}
	
	/**
	 * Create a new color from 4-bit shorthand (up to 0xf)
	 * 
	 * This is used as a shorthand for specifying colors, sort of like abbreviating #ffaabb to #fab in HTML/CSS
	 * @param rChannel
	 * @param gChannel
	 * @param bChannel
	 * @return
	 */
	public static LightsColor new4Bit(int rChannel, int gChannel, int bChannel)
	{
		return new LightsColor(rChannel*128, gChannel*128, bChannel*128);
	}
	
	/**
	 * Get the 11-bit red channel value.
	 * @return
	 */
	public int getR()
	{
		return r;
	}
	
	/**
	 * Get the 11-bit green channel value.
	 * @return
	 */
	public int getG()
	{
		return g;
	}
	
	/**
	 * Get the 11-bit blue channel value.
	 * @return
	 */
	public int getB()
	{
		return b;
	}
}
