using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Collections;
using System.Net.Sockets;
using System.Text;
using UnityStandardAssets.Vehicles.Car;

public class mySocket : MonoBehaviour {

	public CarRemoteControl CarRemoteControl;
	public Camera FrontFacingCamera;
	private CarController _carController;
	private WaypointTracker_pid wpt;

	private string Server_IP = "127.0.0.1";
	const int SERVER_PORT = 15000;
	const int READ_BUFFER_SIZE = 9;
	private byte[] readBuffer = new byte[READ_BUFFER_SIZE];
	const int WRITE_BUFFER_SIZE = 64000;
	private byte[] writeBuffer = new byte[WRITE_BUFFER_SIZE];
	private TcpClient client;

	private int writeBufferSize;

	private bool ReadOk, WriteOk;

	//read from Server
	private void DoRead(IAsyncResult ar)
	{ 
		int BytesRead;
		try
		{
			BytesRead = client.GetStream().EndRead(ar);
			if (BytesRead < 1) 
			{
				// if no bytes were read server has close.  
				Debug.Log("Server DISCONNECTED!");
				return;
			}

			//send flag to main thread
			ReadOk = true;

		} 
		catch
		{
			Debug.Log("Server DISCONNECTED!");
		}
	}

	//write to Server
	private void DoWrite(IAsyncResult ar)
	{ 
		try
		{
			client.GetStream().EndWrite(ar);

			//send flag to main thread
			WriteOk = true;

		} 
		catch
		{
			Debug.Log("Server DISCONNECTED!");
		}
	}



	// Use this for initialization
	void Start () {

		_carController = CarRemoteControl.GetComponent<CarController>();
		wpt = new WaypointTracker_pid ();

		//connect to TCP Server
		ReadOk = false;
		WriteOk = false;
		try 
		{
			client = new TcpClient(Server_IP, SERVER_PORT);
			client.GetStream().BeginRead(readBuffer, 0, READ_BUFFER_SIZE, new AsyncCallback(DoRead), null);
			Debug.Log("Connection OK!");
		}
		catch(Exception ex)
		{
			Debug.Log("Connection ERROR!");
		}
			
	}
	
	// Update is called once per frame
	void Update () {

		//new message received -> decode it and then write back
		if (ReadOk)
		{
			ReadOk = false;

			//decode message from Server
			DecodeMessage();

			//write to Server
			EncodeMessage();
			client.GetStream().BeginWrite(writeBuffer, 0, writeBufferSize, new AsyncCallback(DoWrite), null);
		}
		//message sent correctly -> start reading again
		if (WriteOk)
		{
			WriteOk = false;
			client.GetStream().BeginRead(readBuffer, 0, READ_BUFFER_SIZE, new AsyncCallback(DoRead), null);
		}


	}



	void DecodeMessage() {

		//decode read message

		CarRemoteControl.SteeringAngle = BitConverter.ToSingle(readBuffer, 0);
		CarRemoteControl.Acceleration = BitConverter.ToSingle(readBuffer, 4);

		//restart signal
		bool restartGame = BitConverter.ToBoolean(readBuffer,8);
		if (restartGame) {
			_carController.transform.localPosition = new Vector3(-40.62f,2.6f,108.73f);
			_carController.transform.localEulerAngles = new Vector3(-0.293f,236.078f,0.0f);
		}

	}


	void EncodeMessage() {

		//encoded message to be written

		float cte = wpt.CrossTrackError (_carController);
		writeBuffer[0] = BitConverter.GetBytes(cte)[0];
		writeBuffer[1] = BitConverter.GetBytes(cte)[1];
		writeBuffer[2] = BitConverter.GetBytes(cte)[2];
		writeBuffer[3] = BitConverter.GetBytes(cte)[3];

		float speed = _carController.CurrentSpeed;
		writeBuffer[4] = BitConverter.GetBytes(speed)[0];
		writeBuffer[5] = BitConverter.GetBytes(speed)[1];
		writeBuffer[6] = BitConverter.GetBytes(speed)[2];
		writeBuffer[7] = BitConverter.GetBytes(speed)[3];

		string Image = Convert.ToBase64String(CameraHelper.CaptureFrame(FrontFacingCamera));
		var encoding = Encoding.UTF8;

		encoding.GetBytes(Image,0,Image.Length,writeBuffer,8);

		writeBufferSize = 8+Image.Length;
		//Debug.Log(writeBufferSize);

	}



	void OnApplicationQuit() {
		try {
			client.Close();
		}
		catch{}
	}
}
