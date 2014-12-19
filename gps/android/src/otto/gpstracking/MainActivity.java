package otto.gpstracking;

import java.net.SocketException;
import java.net.UnknownHostException;

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
	TextView text_drone_coordinates, text_phone_coordinates, text_phone_Accuracy, text_seperation;
	
	
	
	public Handler messageHandler;
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		Log.d("message", "onCreate called");
		setContentView(R.layout.activity_main);
		text_drone_coordinates = (TextView)findViewById(R.id.drone_coordinates);
		text_phone_coordinates = (TextView)findViewById(R.id.phone_coordinates);
    	text_phone_Accuracy = (TextView)findViewById(R.id.phone_accuracy);
    	text_seperation = (TextView)findViewById(R.id.seperation);
		
		 messageHandler = new Handler()
		 {
			 String msgText;
			 String displayMessage;
			 Location phoneLocation = null;
			 Location droneLocation = new Location("Drone");
			 
			 
			 @Override
			 public void handleMessage(Message msg)
			 {
				 if(msg.arg1 == 1) //Messages related to network IO
				 {		 
					msgText = ((String)msg.obj);
					
					if(msgText.contains(Messages.ReceiveMessages.Latitude))
					{
						droneLocation.setLatitude(Double.parseDouble(msgText.substring(4)));
						displayMessage = "Latitude: " + droneLocation.getLatitude() + " ";
			
					}else
					{
						
						droneLocation.setLongitude(Double.parseDouble(msgText.substring(4)));
						displayMessage += "Longitude: " + droneLocation.getLongitude();
						text_drone_coordinates.setText(displayMessage);
					}
				 }else if(msg.arg1 == 2) //Receive phone coordinates from LocationListener
				 {
					 phoneLocation = ((Location)msg.obj);
					 

			    	text_phone_coordinates.setText("Lattitude: " + phoneLocation.getLatitude() + "  Longittude" + phoneLocation.getLongitude());
			    	text_phone_Accuracy.setText("Accuracy: " + phoneLocation.getAccuracy() + "m");
					 
				 }
				 
				 if(phoneLocation != null)
				 {
					 float seperationDistance = phoneLocation.distanceTo(droneLocation);
					 text_seperation.setText(String.valueOf(seperationDistance)+"m ");
				 }
				 
				 
			 
			 }
		 };
		
		try {
			udpConnection = new UDP(2550, "192.168.43.84", messageHandler);
		
		} catch (SocketException e) {
			
			e.printStackTrace();
		} catch (UnknownHostException e) {
			
			e.printStackTrace();
		}
	     

		locationManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
        locationListener = new LocationListenerWrapper();
        locationListener.Init(this, udpConnection);
         
   
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
		locationManager.removeUpdates(locationListener);
		Log.d("message", "onPause called");
		
	}
	
	@Override
	public void onResume()
	{
		super.onResume();
		 //update twice a second
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);
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
		
		udpConnection.send(counter + " ");
		counter++;
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
