package otto.gpstracking;


import android.location.*;
import android.os.Bundle;
import android.os.Message;
import android.widget.TextView;
import android.app.Activity;


public class LocationListenerWrapper implements LocationListener {

	Activity activity;
	UDP udpConnection;
	TextView text_phone_coordinates;
	TextView text_phone_Accuracy;
	TextView text_drone_coordinates;

	
	public void Init(Activity mainActivity, UDP myConnection)
	{
		activity = mainActivity;
		udpConnection = myConnection;
		
		

	}
	
	@Override
	public void onLocationChanged(Location location) {
		// TODO Auto-generated method stub
		udpConnection.send("$LAT"+location.getLatitude());
		udpConnection.send("$LON"+location.getLongitude());
  
    	//text_phone_coordinates.setText("Lattitude: " + location.getLatitude() + "  Longittude" + location.getLongitude());
    	//text_phone_Accuracy.setText("Accuracy: " + location.getAccuracy() + "m");
    	
    	Message msg = Message.obtain();
		msg.obj = location;
		msg.arg1 = 2;
		udpConnection.uiHandler.sendMessage(msg);
    	
	}

	@Override
	public void onProviderDisabled(String provider) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onProviderEnabled(String provider) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStatusChanged(String provider, int status, Bundle extras) {
		// TODO Auto-generated method stub
		

	}

}
