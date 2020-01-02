package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

/*
 * class for sending data over UDP
 */
public class UDPDataSender
{   
   //variables
   DatagramSocket socket; 
   InetAddress IPAddress;  
   int port;  
   boolean printSendingData;
   
   public UDPDataSender(String IPAddressString, int port, boolean printSendingData) {
      try {
         socket = new DatagramSocket(port+1);   
         IPAddress = InetAddress.getByName(IPAddressString);
         this.port = port;
         this.printSendingData = printSendingData;
      }
      catch (Exception e) {
         e.printStackTrace();
      }
   }
   
   long lastTime = System.currentTimeMillis(); 
   public void send(String data) {
      if(socket == null)
         return;
      try
      {
         byte[] sendData = data.getBytes();
         DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, port);
         socket.send(sendPacket);  
         if(printSendingData) {
            long time = System.currentTimeMillis(); 
            System.out.println("Sender(" + (time - lastTime) +" ms): " + data);
            lastTime = time;
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}