package otto.gpstracking;

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
				UDP.counter = UDP.milli_delay;
			
				if(send_packets.size() != 0)
				{
					temp = send_packets.poll();
				
					UDP.send_packet_counter++;
					UDP.send_byte_counter +=temp.length();
					
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
		
		byte[] received_data = new byte[16];
			
		public void run()
		{
			String received_string;
			
			
			Log.d("thread", "receive thread started");
			while(udp_on)
			{
				DatagramPacket received= new DatagramPacket(received_data, received_data.length);
				try {
					
					socket.receive(received);
					received_string = new String(received.getData(), 0, received.getLength());
					//received_packets.add(received_string);
					Message msg = Message.obtain();
					msg.obj = received_string.toString();
					msg.arg1 = 1;
					uiHandler.sendMessage(msg);
					
					Log.d("receive", received_string + " "+ received_string.length());
					
					UDP.receive_packet_counter++;
					UDP.receive_byte_counter += received.getLength();
					
				
				} catch (Exception e) {
					
					Log.d("receive", "Failed to receive, IOException was thrown");
					Log.d("receive", e.getLocalizedMessage());
				}
				
			}
			
			Log.d("thread", "receive thread exited");
			
		}

	}

	
	//counts how many packets and bytes are send. They get reset to 0 every second
	static long send_packet_counter = 0; 
	static long send_byte_counter = 0; 
	
	static long receive_packet_counter = 0;
	static long receive_byte_counter = 0;
	
	static long milli_delay = 20; //delay for the send thread
			
	private Send send;
	private Receive receive;

	static long counter = 0;
	boolean udp_on = true;
	
	
	public UDP(int port, String ip, Handler h) throws SocketException, UnknownHostException
	{
		super();
		
		Message msg = Message.obtain();
		
		uiHandler = h;
		msg.arg1 = -1;
		h.sendMessage(msg);
		
		send_semaphore = new Semaphore(1);
		
		socket = new DatagramSocket(port);
		client = InetAddress.getByName(ip);
		
		send = new Send();
		send.start();
		
		receive = new Receive();
		receive.start();

	}
	
	public void send(String data)
	{
		try {
			send_semaphore.acquire();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		send_packets.add(data);
		
		send_semaphore.release();
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
