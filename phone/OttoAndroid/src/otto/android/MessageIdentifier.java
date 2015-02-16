package otto.android;


//Message Identifiers
public class MessageIdentifier {
	
	
	public static class Receive
	{
		public static final String Latitude = "$LAT"; 	//Received drone latitude
		public static final String Longitude = "$LON";	//Received drone longitude
		public static final String BatteryStatus = "$BTS";
		public static final String Altitude = "$ALT";
		
	}
	
	public static class Send
	{
		public static final String Latitude = "$LAT"; 	//Send user latitude
		public static final String Longitude = "$LON";	//Send user longitude
		public static final String TakeOff = "$TKF";
		public static final String Land = "$STP";

		/*public static final byte Distance_Close = "$DCL";	//Seperation distance close, empty payload
		public static final byte Distance_Near = "$DNR";	//Seperation distance medium, empty payload
		public static final String Distance_Far = "$DFR";	//Seperation distance far, empty payload
		public static final String TakeOff = "$TKF";	//Take off command, empty payload
		public static final String ACK = "$ACK";	//ACK message, no payload*/
	}
	
	public static class UIHandler
	{
		public static final int NetworkIO = 1;
		public static final int Location = 2;
	
	}
	

}
