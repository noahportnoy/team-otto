package otto.android;

public class OttoPacket {
	
	private String _packet;
	

	public OttoPacket(String packet)
	{
		_packet = packet;
	}
	
	public OttoPacket(String identifier, String payload)
	{
		_packet = identifier+payload+"#";
	}
	
	public String getIdentifier()
	{
		return _packet.substring(0, 3); // first 4 bytes are identifier
	}
	
	public String getPayload()
	{
		return _packet.substring(4, _packet.length());
	}
	
	

}
