package otto.android;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.Semaphore;

import android.os.Handler;
import android.os.Message;
import android.util.Log;

public class UDP{
	
	private DatagramSocket socket = null;
	private InetAddress client = null;
	
	public Handler uiHandler;
	
	private Semaphore send_semaphore;
	
	//Store all our data
	LinkedBlockingQueue<String> send_packets = new LinkedBlockingQueue<String>();
	LinkedBlockingQueue<String> received_packets = new LinkedBlockingQueue<String>();
	
	//Helper classes
	private class Send extends Thread
	{
		int port = 0; //the remote port
		
		public Send()
		{
			this.port = socket.getLocalPort(); //port of this machine and to the remote machine
		}
		
		public void run()
		{
			Log.d("thread", "send thread started");
			DatagramPacket packet;
			String temp;
			
			while(udp_on)
			{
			
				if(send_packets.size() != 0)
				{
					temp = send_packets.poll();
					
					packet =  new DatagramPacket(temp.getBytes(), temp.length(), client, port);
					Log.d("send", temp);
					try 
					{
						socket.send(packet);
					
					} catch (IOException e) {
						
						Log.d("send", "Packet with data " + temp + " failed to send");
					}
		 
				}
				
				 try {
					Thread.sleep(UDP.milli_delay);
				} catch (InterruptedException e) {
				
					e.printStackTrace();
				}
				 
				
			}
			 
			Log.d("thread", "send thread exited");
			
		}
		
			
	}

	private class Receive extends Thread
	{
		
		byte[] received_data = new byte[32];
			
		public void run()
		{
			Log.d("thread", "receive thread started");
			
			String received_string;
			
			while(udp_on)
			{
				DatagramPacket receivedPacket= new DatagramPacket(received_data, received_data.length);
				
				try {
					
					socket.receive(receivedPacket);
					received_string = new String(receivedPacket.getData(), 0, receivedPacket.getLength());

					Message msg = Message.obtain(); //Obtain reference
					msg.what = MessageIdentifier.UIHandler.NetworkIO; //identify the message
					msg.obj = received_string.toString(); //contents of the message
					uiHandler.sendMessage(msg);
					
					Log.d("receive", received_string + " "+ received_string.length());
					
					
				
				} catch (Exception e) {
					
					Log.d("receive", "Failed to receive, IOException was thrown");
					Log.d("receive", e.getLocalizedMessage());
				}
				
			}
			
			Log.d("thread", "receive thread exited");
			
		}

	}

	
	static long milli_delay = 20; //delay for the send thread
			
	private Send send;
	private Receive receive;

	boolean udp_on = true;
	
	
	public UDP(int port, String ip, Handler h) throws SocketException, UnknownHostException
	{
		super();
		
		
		//message handler from the main thread
		uiHandler = h;
		
		send_semaphore = new Semaphore(1);
		
		socket = new DatagramSocket(port);
		client = InetAddress.getByName(ip);
		
		send = new Send();
		send.start();
		
		receive = new Receive();
		receive.start();

	}
	
	public void send(String messageIdentifier, String messagePayload)
	{
		
		
		try {
			send_semaphore.acquire();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		send_packets.add(messageIdentifier + messagePayload);
		
		send_semaphore.release();
	}
	
	public void send(String data)
	{
		send("" ,data);
	}
	
	public void close()
	{
		socket.close();
		udp_on = false;
		try {
			receive.join(1000);
			send.join(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	
	}
	
	
}
