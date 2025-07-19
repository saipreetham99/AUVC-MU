// AUVManager.cs (Corrected)
using UnityEngine;
using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.IO;
using Mujoco;

public class AUVManager : MonoBehaviour
{
  // --- Public Configuration ---
  [Header("Network Settings")]
  public string GsServerIP = "127.0.0.1";
  public int GsServerPort = 8080;
  public string RpiServerIP = "192.168.2.10";
  public int RpiServerPort = 8081;

  [Header("Physics Settings")]
  public bool enableBuoyancyPhysics = true;
  public float waterGain = 600.0f; // Increased for more noticeable effect
  public float volumeDisplaced = 0.045f; // m^3, Adjusted for better floating
  public float startHeight = -0.5f; // The target water level height
  public int subBodyId = 1; // The Mujoco body ID for the submarine

  // --- State Enums ---
  public enum Command : int { HEARTBEAT = 0, RESET = 1, APPLY_CTRL = 3, GET_SENSOR_DATA = 4, GET_IMAGE = 5 }
  public enum Response : int { NO_ERROR = 0, ERROR = 1, LIST_OF_DATA = 3 }

  // --- Internal State ---
  private volatile bool _isRpiConnected = false;
  private Thread _gsServerThread;
  private Thread _rpiClientThread;
  private volatile bool _stopThreads = false;

  // Data synchronization between threads
  private class GsCommand
  {
    public Command CommandType;
    public int StepsToRun;
    public float[] Payload;
  }
  private volatile GsCommand _commandToProcess = null;
  private readonly object _commandLock = new object();

  private volatile int _stepsCounter = 0;
  private ManualResetEvent _stepsCompletedEvent = new ManualResetEvent(false);
  private volatile byte[] _responseToSend = null;

  private readonly ConcurrentQueue<float[]> _rpiSensorDataQueue = new ConcurrentQueue<float[]>();

  private MjScene _mjScene;

  void Start()
  {
    _mjScene = MjScene.Instance;
    if (_mjScene == null)
    {
      Debug.LogError("MjScene instance not found!");
      return;
    }

    // Register the control callback for applying forces
    _mjScene.ctrlCallback += OnControlCallback;
    Debug.Log("AUVManager started and callback registered.");

    // Start networking threads
    _gsServerThread = new Thread(GsServerLoop);
    _gsServerThread.IsBackground = true;
    _gsServerThread.Start();

    _rpiClientThread = new Thread(RpiClientLoop);
    _rpiClientThread.IsBackground = true;
    _rpiClientThread.Start();
  }

  void OnDestroy()
  {
    _stopThreads = true;
    _gsServerThread?.Join();
    _rpiClientThread?.Join();
    if (_mjScene != null)
    {
      _mjScene.ctrlCallback -= OnControlCallback;
    }
    Debug.Log("AUVManager stopped and threads cleaned up.");
  }

  void FixedUpdate()
  {
    // This function runs on the main Unity thread in sync with physics
    lock (_commandLock)
    {
      if (_commandToProcess != null)
      {
        if (_stepsCounter < _commandToProcess.StepsToRun)
        {
          _stepsCounter++;
        }
        else
        {
          using (var ms = new MemoryStream())
            using (var writer = new BinaryWriter(ms))
            {
              writer.Write((int)Response.NO_ERROR);
              _responseToSend = ms.ToArray();
            }
          _stepsCompletedEvent.Set();
          _commandToProcess = null;
        }
      }
    }
  }

  private unsafe void OnControlCallback(object sender, MjStepArgs args)
  {
    // This is the central point for applying all physics for this step.
    // It's called automatically between mj_step1 and mj_step2.
    if (args.data == null || args.model == null) return;

    // 1. ZERO OUT ALL FORCES from the previous step. This is crucial.
    int nv = args.model->nv;
    for (int i = 0; i < nv; i++)
    {
      args.data->qfrc_applied[i] = 0.0;
    }
    int nbody = args.model->nbody;
    for (int i = 0; i < nbody * 6; i++)
    {
      args.data->xfrc_applied[i] = 0;
    }

    // 2. Apply continuous physics like buoyancy.
    if (enableBuoyancyPhysics)
    {
      ApplyBuoyancy(args.model, args.data);
    }

    // 3. Apply event-based forces like thruster commands from the client.
    lock (_commandLock)
    {
      if (_commandToProcess != null && _commandToProcess.CommandType == Command.APPLY_CTRL)
      {
        ApplyThrusterForces(args.model, args.data, _commandToProcess.Payload);
      }
    }
  }

  // --- Ground Station Server Logic (Unchanged) ---
  private void GsServerLoop()
  {
    TcpListener server = null;
    try
    {
      server = new TcpListener(IPAddress.Parse(GsServerIP), GsServerPort);
      server.Start();
      Debug.Log($"GS Server listening on {GsServerIP}:{GsServerPort}");

      while (!_stopThreads)
      {
        using (TcpClient client = server.AcceptTcpClient())
        {
          Debug.Log("GS Python Client connected.");
          using (NetworkStream stream = client.GetStream())
          {
            while (!_stopThreads && client.Connected)
            {
              if (!client.Client.Poll(1000, SelectMode.SelectRead) || client.Available == 0) continue;

              var reader = new BinaryReader(stream);
              var commandType = (Command)reader.ReadInt32();
              var steps = reader.ReadInt32();
              var payloadLength = reader.ReadInt32();
              var payload = new float[payloadLength];
              for (int i = 0; i < payloadLength; i++) { payload[i] = reader.ReadSingle(); }

              lock (_commandLock)
              {
                _commandToProcess = new GsCommand { CommandType = commandType, StepsToRun = steps, Payload = payload };
                _stepsCounter = 0;
              }
              _stepsCompletedEvent.Reset();
              _stepsCompletedEvent.WaitOne();

              if (_responseToSend != null)
              {
                stream.Write(_responseToSend, 0, _responseToSend.Length);
                _responseToSend = null;
              }
            }
          }
          Debug.Log("GS Python Client disconnected.");
        }
      }
    }
    catch (ThreadAbortException) { /* Do Nothing */ }
    catch (Exception e) { Debug.LogError($"GS Server Error: {e.Message}"); }
    finally { server?.Stop(); }
  }


  // --- RPi Client Logic (Unchanged) ---
  private void RpiClientLoop()
  {
    while (!_stopThreads)
    {
      try
      {
        using (var rpiClient = new TcpClient())
        {
          rpiClient.Connect(RpiServerIP, RpiServerPort);
          if (rpiClient.Connected)
          {
            _isRpiConnected = true;
            Debug.Log($"Successfully connected to RPi at {RpiServerIP}:{RpiServerPort}");
            using (var stream = rpiClient.GetStream())
            {
              var writer = new BinaryWriter(stream);
              var reader = new BinaryReader(stream);

              while (!_stopThreads && rpiClient.Connected)
              {
                writer.Write((int)Command.HEARTBEAT);
                writer.Flush();
                Thread.Sleep(5000);
              }
            }
          }
        }
      }
      catch (Exception) { /* Connection failed */ }
      finally
      {
        if (_isRpiConnected) Debug.Log("Disconnected from RPi.");
        _isRpiConnected = false;
        Thread.Sleep(5000);
      }
    }
  }

  // --- Physics Application Logic ---
  private unsafe void ApplyBuoyancy(MujocoLib.mjModel_* model, MujocoLib.mjData_* data)
  {
    // Get the submarine body's vertical position from the free joint
    double height = data->qpos[2]; // Assumes qpos[2] is Z (vertical) of the free joint
    float maxBuoyancyForce = 9.806f * volumeDisplaced * 1000; // F_buoyancy = rho_water * V_displaced * g
    float forceError = (startHeight - (float)height) * waterGain;
    float forceClamped = Mathf.Clamp(forceError, -maxBuoyancyForce, maxBuoyancyForce);

    // Apply the buoyant force in the global +Z direction at the body's center of mass.
    // xfrc_applied is a Cartesian force vector [fx, fy, fz, tx, ty, tz] applied to the body COM.
    data->xfrc_applied[subBodyId * 6 + 2] += forceClamped;
  }

  private unsafe void ApplyThrusterForces(MujocoLib.mjModel_* model, MujocoLib.mjData_* data, float[] motorCommands)
  {
    // Assumes 6 thrusters and motorCommands contains 6 values.
    // Assumes you have sites named "thruster1", "thruster2", etc. in your XML.
    for (int i = 0; i < motorCommands.Length; i++)
    {
      int siteId = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_SITE, $"thruster{i + 1}");
      if (siteId != -1)
      {
        double forceMagnitude = motorCommands[i];
        double* siteMatrix = data->site_xmat + 9 * siteId; // 3x3 rotation matrix for the site

        // **FIX**: Manual rotation of the local force vector [0, 0, forceMagnitude]
        // The local Z-axis of the site corresponds to the third column of its rotation matrix.
        double[] worldForce = new double[3];
        worldForce[0] = siteMatrix[2] * forceMagnitude; // R02 * Fz
        worldForce[1] = siteMatrix[5] * forceMagnitude; // R12 * Fz
        worldForce[2] = siteMatrix[8] * forceMagnitude; // R22 * Fz

        // Apply this world-space force at the site's world-space position.
        // mj_applyFT converts this to generalized forces (qfrc_applied) for you.
        fixed (double* f = worldForce, t = new double[3] { 0, 0, 0 })
        {
          double* point = data->site_xpos + 3 * siteId;
          MujocoLib.mj_applyFT(model, data, f, t, point, subBodyId, data->qfrc_applied);
        }
      }
    }
  }
}
