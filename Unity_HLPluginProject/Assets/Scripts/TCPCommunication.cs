using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

#if !UNITY_EDITOR
    using Windows.Networking;
    using Windows.Networking.Sockets;
    using Windows.Storage.Streams;
#endif

//Able to act as a reciever 
public class TCPCommunication : MonoBehaviour
{
    //public Text debug;
    public static String message = "";
    static string dataToSend = ""; // message sent to pc
    static bool sendMessage = false;

#if !UNITY_EDITOR
    StreamSocket socket;
    StreamSocketListener listener;
    DataWriter dw;
#endif

    // Use this for initialization
    void Start()
    {
#if !UNITY_EDITOR
        listener = new StreamSocketListener();
        listener.ConnectionReceived += Listener_ConnectionReceived;
        listener.Control.KeepAlive = false;

        Listener_Start();
#endif
    }

#if !UNITY_EDITOR
    private async void Listener_Start()
    {
        Debug.Log("Listener started");
        try
        {
            await listener.BindServiceNameAsync("12345");
        }
        catch (Exception e)
        {
            Debug.Log("Error: " + e.Message);
        }

        Debug.Log("Listening");
    }

    private async void Listener_ConnectionReceived(StreamSocketListener sender, StreamSocketListenerConnectionReceivedEventArgs args)
    {
        Debug.Log("Connection received");
        var streamReader = new StreamReader(args.Socket.InputStream.AsStreamForRead());
        dw = new DataWriter(args.Socket.OutputStream);
        try
        {
            while (true)
            {
                string getin = await streamReader.ReadLineAsync();  // string    
                if (getin != "" && getin != null)
                {
                    message = getin;
                    Debug.Log("latest message: " + message);
                }

            }
        }
        catch (Exception e)
        {
            Debug.Log("disconnected!!!!!!!! " + e);
        }

    }


    // send message
    public async void Send(string sendmessage)
    {
        try
        {
            dw.WriteString(sendmessage + "\n");
            await dw.StoreAsync();  // Commits data in the buffer to the output stream
            Debug.Log("actually sent");

        }
        catch (Exception e)
        {
            Debug.Log("can't send to Hololens");
        }
    }

#endif

    void Update()
    {
#if !UNITY_EDITOR
        if (sendMessage)
        {
            Send(dataToSend);
            sendMessage = false;
            dataToSend = "";
        }
#endif

        //if (message != "")
        //{
        //    debug.text = message;
        //}
    }

    public static void SetMessage(string data)
    {
        sendMessage = true;
        dataToSend = data;
    }

}
