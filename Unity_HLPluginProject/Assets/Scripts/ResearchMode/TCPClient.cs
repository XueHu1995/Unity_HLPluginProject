using System;
using UnityEngine;
//using System.Runtime.Serialization.Formatters.Binary;
#if WINDOWS_UWP
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif

public class TCPClient : MonoBehaviour
{
    #region Unity Functions
    private void Awake()
    {
#if WINDOWS_UWP
        StartCoonection();
#endif
    }
    private void OnApplicationFocus(bool focus)
    {
        if (!focus)
        {
#if WINDOWS_UWP
            StopCoonection();
#endif
        }
    }
    #endregion // Unity Functions

    private void Start()
    {

    }

    [SerializeField]
    string hostIPAddress, port;

    public string targetIP;
    public string targetPort;

#if WINDOWS_UWP
    StreamSocket socket = null;
    public DataWriter dw;
    public DataReader dr;
    private async void StartCoonection()
    {
        try
        {
            hostIPAddress = targetIP; port = targetPort;
            if (socket == null)
            {
                socket = new StreamSocket();
                var hostName = new Windows.Networking.HostName(hostIPAddress);
                await socket.ConnectAsync(hostName, port);
            }
            if (dw == null)
            {
                dw = new DataWriter(socket.OutputStream);
            }
            if (dr == null)
            {
                dr = new DataReader(socket.InputStream);
                dr.InputStreamOptions = InputStreamOptions.Partial;
            }
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
    }

    private void StopCoonection()
    {
        dw?.DetachStream();
        dw?.Dispose();
        dw = null;

        dr?.DetachStream();
        dr?.Dispose();
        dr = null;
        
        socket?.Dispose();
    }

    bool lastMessageSent = true;
    public async void SendUINT16Async(ushort[] data)
    {
        if (!lastMessageSent) return;
        lastMessageSent = false;
        try
        {
            // Write header
            dw.WriteString("s"); // header "s" stands for it is ushort array (uint16)
            
            // Write length and data
            dw.WriteInt32(data.Length);
            dw.WriteBytes(UINT16ToBytes(data));
            
            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        lastMessageSent = true;
    }

    public async void SendUINT16Async(ushort[] data1, ushort[] data2)
    {
        if (!lastMessageSent) return;
        lastMessageSent = false;
        try
        {
            // Write header
            dw.WriteString("s"); // header "s" stands for it is ushort array (uint16)

            // Write Length
            dw.WriteInt32(data1.Length + data2.Length);

            // Write actual data
            dw.WriteBytes(UINT16ToBytes(data1));
            dw.WriteBytes(UINT16ToBytes(data2));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        lastMessageSent = true;
    }

    public async void SendUINT16AsyncWithMatrix(ushort[] data1, ushort[] data2, float[] mat)
    {
        if (!lastMessageSent) return;
        lastMessageSent = false;
        try
        {
            // Write header
            dw.WriteString("s"); // header "s" stands for it is ushort array (uint16)

            // Write Length
            dw.WriteInt32(data1.Length + data2.Length);

            // Write actual data
            dw.WriteBytes(UINT16ToBytes(data1));
            dw.WriteBytes(UINT16ToBytes(data2));
            dw.WriteBytes(FloatToBytes(mat));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        lastMessageSent = true;
    }

#endif

    #region Helper Function
    byte[] UINT16ToBytes(ushort[] data)
    {
        byte[] ushortInBytes = new byte[data.Length * sizeof(ushort)];
        System.Buffer.BlockCopy(data, 0, ushortInBytes, 0, ushortInBytes.Length);
        return ushortInBytes;
    }

    byte[] FloatToBytes(float[] data)
    {
        byte[] floatInBytes = new byte[data.Length * sizeof(int)];
        int[] floatValuesInt = new int[data.Length];
        
        // int conversion
        for (int i = 0; i < data.Length; i++)
        {
            floatValuesInt[i] = (int)(data[i] * 10000);
        }

        System.Buffer.BlockCopy(floatValuesInt, 0, floatInBytes, 0, floatInBytes.Length);
        return floatInBytes;
    }
#endregion
}
