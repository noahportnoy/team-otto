package otto.android;


import android.location.*;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.widget.TextView;


public class LocationListenerWrapper implements LocationListener {

	UDP udpConnection;
	Handler uiHandler;

	
	
	public void Init(UDP myConnection, Handler uiHandler)
	{
		this.udpConnection = myConnection;	
		this.uiHandler = uiHandler;
		
	}
	
	@Override
	public void onLocationChanged(Location location) {
		
		//Send phone coordinates to rasp pi
		udpConnection.send(MessageIdentifier.Send.Latitude, String.valueOf(location.getLatitude()));
		udpConnection.send(MessageIdentifier.Send.Longitude, String.valueOf(location.getLongitude()));
		
		//Send phone coordinates to UI thread
    	Message msg = Message.obtain();
    	msg.what = MessageIdentifier.UIHandler.Location;
		msg.obj = location;
		uiHandler.sendMessage(msg);
    	
	}

	@Override
	public void onProviderDisabled(String provider) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onProviderEnabled(String provider) {
		
		
	}

	@Override
	public void onStatusChanged(String provider, int status, Bundle extras) {
		// TODO Auto-generated method stub
		

	}

}
