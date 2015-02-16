package otto.android;

import java.net.SocketException;
import java.net.UnknownHostException;

import otto.android.R;
import android.app.Activity;
import android.content.Context;
import android.location.Location;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.TextView;


public class MainActivity extends Activity {

	LocationManager locationManager;
	LocationListenerWrapper locationListener;
	UDP udpConnection;
	int counter = 0;
	TextView text_phone_lat, 
				text_phone_lon,
				text_drone_lat,
				text_drone_lon,
				text_phone_Accuracy,
				text_battery_status,
				text_altitude;// text_seperation;
	
	//Handle message from other threads
	public Handler uiHandler;
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		
		super.onCreate(savedInstanceState);
		
		
		Log.d("message", "onCreate called");
		setContentView(R.layout.activity_main);
		
		text_drone_lat = (TextView)findViewById(R.id.drone_lat);
		text_drone_lon = (TextView)findViewById(R.id.drone_lon);
		text_phone_lat = (TextView)findViewById(R.id.phone_lat);
		text_phone_lon = (TextView)findViewById(R.id.phone_lon);
    	text_phone_Accuracy = (TextView)findViewById(R.id.phone_accuracy);
    	text_battery_status = (TextView)findViewById(R.id.battery_status);
    	text_altitude = (TextView)findViewById(R.id.altitude);
    	//text_seperation = (TextView)findViewById(R.id.seperation);
		
		uiHandler = new Handler()
		 {
			String networkMessage;
			Location phoneLocation = null;
			Location droneLocation = new Location("Drone");
		 
		 
			//Messages from other threads
		 @Override
	 		public void handleMessage(Message msg)
			 {
				 switch(msg.what)
				 {
				 	//Data from Network IO threads
					 case MessageIdentifier.UIHandler.NetworkIO:
					 {
						 networkMessage = ((String)msg.obj);
						 networkMessage = networkMessage.replace("!", "0"); //Remove end of message character
						
						if(networkMessage.contains(MessageIdentifier.Receive.Latitude))
						{
							droneLocation.setLatitude(Double.parseDouble(networkMessage.substring(4)));
							text_drone_lat.setText("Lat:" + droneLocation.getLatitude());
				
						}else if(networkMessage.contains(MessageIdentifier.Receive.Longitude))
						{
							
							droneLocation.setLongitude(Double.parseDouble(networkMessage.substring(4)));
							text_drone_lon.setText("Lon:"+droneLocation.getLongitude());
							
						}else if(networkMessage.contains(MessageIdentifier.Receive.BatteryStatus))
						{
							text_battery_status.setText("" +Double.parseDouble(networkMessage.substring(4) ));
						}
						else if(networkMessage.contains(MessageIdentifier.Receive.Altitude))
						{
							text_altitude.setText("" +Double.parseDouble(networkMessage.substring(4) ));
						}

						else
						{
							Log.d("debug", "Network received unknown message identifier ");
						}
					 }
					 	break;
				 
					 //Data from Location threads
					 case MessageIdentifier.UIHandler.Location:
					 {
						 
						phoneLocation = ((Location)msg.obj);
			
				    	text_phone_lat.setText("Lat:" + phoneLocation.getLatitude());
				    	text_phone_lon.setText("Lon:" + phoneLocation.getLongitude());
				    	text_phone_Accuracy.setText("Accuracy: " + phoneLocation.getAccuracy() + "m");
					 }
					 	break;
					 
					 default:
						 Log.d("debug", "UI Handler received unknown message identifier");
						 break;
						 
				 }
				 
				 
				 if(phoneLocation != null)
				 {
					 float seperationDistance = phoneLocation.distanceTo(droneLocation);
					// text_seperation.setText(String.valueOf(seperationDistance)+"m ");
				 }
				 
				 }
		 };

		 //Open UDP connection
		try {
			udpConnection = new UDP(2550, "192.168.43.84", uiHandler);
		
		} catch (SocketException e) {
			
			e.printStackTrace();
		} catch (UnknownHostException e) {
			
			e.printStackTrace();
		}
	     

		//Set up LocationListener using GPS
		locationManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
        locationListener = new LocationListenerWrapper();
        locationListener.Init(udpConnection, uiHandler);
         
   
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}
	
	@Override
	public void onPause()
	{
		super.onPause();
		//Suspend Location updates
		locationManager.removeUpdates(locationListener);
		Log.d("message", "onPause called");
		
	}
	
	@Override
	public void onResume()
	{
		super.onResume();
		 //update twice a second
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 100, 0.0f, locationListener);
		Log.d("message", "onResume called");
	}
	
	@Override
	public void onDestroy()
	{
		super.onDestroy();
		
		udpConnection.close();
	
		Log.d("message", "onDestroy called");
	
	}
	
	
	public void button1Click(View v)
	{
		
		udpConnection.send("", counter + " ");
		counter++;
	}
	
	public void testOnClick(View v)
	{
		udpConnection.send(MessageIdentifier.Send.TakeOff, "");
	}
	
	public void onClickPiShut(View v)
	{
		udpConnection.send("$PSH", "");
	}
	

	public void onClickStop(View v)
	{
		udpConnection.send(MessageIdentifier.Send.Land, "");
	}
	
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle action bar item clicks here. The action bar will
		// automatically handle clicks on the Home/Up button, so long
		// as you specify a parent activity in AndroidManifest.xml.
		int id = item.getItemId();
		if (id == R.id.action_settings) {
			return true;
		}
		return super.onOptionsItemSelected(item);
	}

}
