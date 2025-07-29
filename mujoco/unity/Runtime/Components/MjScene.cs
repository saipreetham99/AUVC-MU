// Copyright 2019 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;
using System.Text;
using System.Xml;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.UI;
using TMPro;
namespace Mujoco {

  public class PhysicsRuntimeException : Exception {
    public PhysicsRuntimeException(string message) : base(message) {}
  }

  [RequireComponent(typeof(SliderController))]
  public class MjScene : MonoBehaviour {
    public SliderController sliderController;
    public TMP_InputField waterSurfaceHeight;
    public TMP_InputField forceMultiplier;
    public TMP_InputField viscosityTextField;
    public TMP_InputField timestepTextField;
    public TMP_InputField realtimeTextField;
    public TMP_InputField simtimeTextField;
    public TMP_InputField distanceTextField;
    // public ThreadSafeBoundingBoxProvider targetSubmarine;


    // Timing behaviour
    private MultiStopwatch timer = new MultiStopwatch();

    public unsafe MujocoLib.mjModel_* Model = null;
    public unsafe MujocoLib.mjData_* Data = null;
    public float previousValue = 0;
    // Thread-safe storage for control forces
    private bool hasControlForces = false; // Does not need to be volatile, since lock holds it down well
    private readonly object serverSimulationLock = new object();
    private readonly Queue<int> pendingSteps = new Queue<int>();
    private readonly Queue<float[]> pendingControlForces = new Queue<float[]>();
    private float[] _currentControlForces = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };


    // Force Rendering/Force Application
    public int[] siteCtrlIds = new int[] { 5, 6, 7, 8, 9, 10 }; // a1-a6
    [SerializeField]
    private List<ArrowController> ArrowControllers;

    // Socket server configuration
    [Header("Socket Server Settings")]
    public bool enableSocketServer = true;
    public string serverIpAddress = "127.0.0.1";


    const int MAX_SENSOR_COUNT= 5; // TODO: IMU and Depth for a single sub (Double it! Second sub on the way!)
    private static float[] _sensorData = new float[MAX_SENSOR_COUNT];
    private static byte[] _sensorDataBytes = new byte[MAX_SENSOR_COUNT * sizeof(float)];

    private static byte[] modelInfoBuffer = new byte[6 * sizeof(float)];
    public int numSteps = 0;

    public volatile int isSync = 1;

    public enum MsgHeader {
      ERROR = 0,
      NO_ERROR = 1,
      HEARTBEAT = 2,
      GET_MODEL_INFO = 3,
      GET_SENSORDATA = 4,
      GET_RGB_IMAGE = 5,
      GET_MASKED_IMAGE = 6,
      APPLY_CTRL = 7,
      STEP_SIM = 8,
      RESET = 9,
    }

    public struct CommandInfo {
      public MsgHeader MsgHeader;
      public string Name;
      public string HelpString;
      public int ExpectedRecvBytes;  // Expected bytes to receive from client
      public int ExpectedSendBytes;  // Expected bytes to send to client
      public float ExpectedTimeoutSeconds; // Expected timeout for this command
      public int HelpStringMessageLength; // Length of HelpString in bytes (UTF-8 encoded)

      public CommandInfo(MsgHeader msgHeader, string name, string helpString, int expectedRecvBytes, int expectedSendBytes, float expectedTimeoutSeconds = 1.0f) {
        MsgHeader = msgHeader;
        Name = name;
        HelpString = helpString;
        ExpectedRecvBytes = expectedRecvBytes;
        ExpectedSendBytes = expectedSendBytes;
        ExpectedTimeoutSeconds = expectedTimeoutSeconds;
        HelpStringMessageLength = System.Text.Encoding.UTF8.GetByteCount(helpString); // UTF-8 byte count
      }
    }

    private static readonly CommandInfo[] Commands = {
      new CommandInfo(
            MsgHeader.ERROR,                            // ENUM HEADER CODE
            "ERROR",                                    // Name
            $"<ret_only:{(int)MsgHeader.ERROR}>\n",     // Help String
            0,                                          // Expected Recv Bytes
            1,                                          // Expected Send Bytes
            0.1f                                        // Expected Timeout
          ),
      new CommandInfo(
           MsgHeader.NO_ERROR,                         // ENUM HEADER CODE
           "NO_ERROR",                                 // Name
           $"<ret_only:{(int)MsgHeader.NO_ERROR}>\n",  // Help String
           0,                                          // Expected Recv Bytes
           1,                                          // Expected Send Bytes
           0.1f                                        // Expected Timeout
          ),
      new CommandInfo(
            MsgHeader.HEARTBEAT,                          // ENUM HEADER CODE
            "HEARTBEAT",                                  // Name
            $"[send: {(int)MsgHeader.HEARTBEAT}.0f],\n"+  // Help String
             "recv: status_byte\n",
            4,                                            // Expected Recv Bytes
            1,                                            // Expected Send Bytes
            0.1f                                          // Expected Timeout
          ),
      new CommandInfo(
            MsgHeader.GET_MODEL_INFO,                         // ENUM HEADER CODE
            "GET_MODEL_INFO",                                 // Name
            $"[send: {(int)MsgHeader.GET_MODEL_INFO}.0f]\n,"+ // Help String
             " recv: [<d.time>,"+
             " <m.nq>,"+
             " <m.nv>,"+
             " <m.na>,"+
             " <m.nu>,"+
             " <m.nbody>]\n",
            4,                                                // Expected Recv Bytes
            24,                                               // Expected Send Bytes
            0.1f                                              // Expected Timeout
          ),
      new CommandInfo(
          MsgHeader.GET_SENSORDATA,
          "GET_SENSORDATA",
          $"[send: {(int)MsgHeader.GET_SENSORDATA}.0f],\n"+
           " recv:[<imu.w>, <imu.x>, <imu.y>, <imu.z>, <time>, <depth>, <bbox.x>, <bbox.y>, <bbox.w>, <bbox.h>]\n",
          4,                                               // Expected Recv Bytes
          40,                                              // Expected Send Bytes: 10 floats * 4 bytes/float
          2.0f
      ),
      // new CommandInfo(
      //       MsgHeader.GET_SENSORDATA,                         // ENUM HEADER CODE
      //       "GET_SENSORDATA",                                 // Name
      //       $"[send: {(int)MsgHeader.GET_SENSORDATA}.0f],\n"+ // Help String
      //        " recv:[<sub1:imu.w>,"+
      //        " <sub1:imu.x>,"+
      //        " <sub1:imu.y>,"+
      //        " <sub1:imu.z>],\n"+
      //        // " recv: <sub2:imu.w>,"+
      //        //       " <sub2:imu.x>,"+
      //        //       " <sub2:imu.y>,"+
      //        //       " <sub2:imu.z]\n"
      //        " recv:[<d.time>],\n",
      //        4,                                               // Expected Recv Bytes
      //        20,                                              // Expected Send Bytes: 5 floats * 4 bytes/float
      //        2.0f                                             // Expected Timeout
      //     ),
      new CommandInfo(
            MsgHeader.GET_RGB_IMAGE,                          // ENUM HEADER CODE
            "GET_RGB_IMAGE",                                  // Name
            $"[send: {(int)MsgHeader.GET_RGB_IMAGE}.0f],\n"+  // Help String
             " recv: <[r][g][b] pixels, row order>]\n",
            4,                                                // Expected Recv Bytes
            4096,                                             // TODO: Expected Recv Bytes
            2.0f                                              // Expected Timeout
          ),
      new CommandInfo(
            MsgHeader.GET_MASKED_IMAGE,                         // ENUM HEADER CODE
            "GET_MASKED_IMAGE",                                 // Name
            $"[send: {(int)MsgHeader.GET_MASKED_IMAGE}.0f],\n"+ // Help String
             " recv: [<[r][g][b] pixels, row order>]\n",
            4,                                                  // Expected Recv Bytes
            1,                                                  // Expected Send Bytes
            2.0f                                                // Expected Timeout
          ),
      new CommandInfo(
            MsgHeader.APPLY_CTRL,                                // ENUM HEADER CODE
            "APPLY_CTRL",                                        // Name
            $"[idx:0][send: {(int)MsgHeader.APPLY_CTRL}.0f],\n"+ // Help String
             " [idx:1]<numSteps> (Simulate n steps after "+
                                  "applying control),\n"+
             " [idx:2]<sync/Async> default=1(Sync/Async)\n"+
             " [idx:3-8]f1,f2,f3,f4,f5,f6],\n"+
             " recv: status_byte + sensor_data + time + depth + bbox\n",
            32,                                                 // Expected Revc Bytes
            21,                                                 // Expected Send Bytes: 1 status byte + 5 floats sensor data
            1.0f                                                // Expected Timeout
          ),
      new CommandInfo(
            MsgHeader.STEP_SIM,                                   // ENUM HEADER CODE
            "STEP_SIM",                                           // Name
            $"[idx:0][send: {(int)MsgHeader.STEP_SIM}.0f],\n"+    // Help String
             " [idx:1]<numSteps>default=1 (Simulate n steps),\n"+
             " [idx:2]<sync/async>default=1 (Sync/Async),\n"+
             " recv: status_byte + sensor_data + time + depth + bbox\n",
            8,                                                    // Expected Revc Bytes
            21,                                                   // Expected Send Bytes: 1 status byte + 5 floats sensor data
            1.0f                                                  // Expected Timeout
          ),
      new CommandInfo(
            MsgHeader.RESET,                                            // ENUM HEADER CODE
            "RESET",                                                    // Name
            $"[idx:0][send: {(int)MsgHeader.RESET}.0f],\n"+             // Help String
             "[idx:1]<numSteps(default=0)>,\n"+
             "[idx:2-8]<sub1: pose array [Px,Py,Pz,w,x,y,z]>,"+
             "[idx:9-11] <vel [LinVelx, LinVely, LinVelz]>]"+
             "[idx:12-14] <ang vel [AngVelx, AngVely, AngVelz]>]\n"+
             " recv: status_byte + sensor_data + time + depth + bbox\n",
            60,                                                         // Expected Revc Bytes
            21,                                                         // Expected Send Bytes: 1 status byte + 5 floats sensor data
            2.0f                                                        // Expected Timeout
          ),
    };

    public static int NetHelpMessageLength =0;


    // Common Socket structure to avoid code duplication
    private class SocketData
    {
      public Socket socket;
      public int port;
      public bool isRunning;
      public Thread thread;
      public string name;
    }

    // Server variables with static initialization
    private SocketData ImageSocket = new SocketData {
      port = 60000,
      isRunning = false,
      name = "Observation"
    };

    private SocketData actionSocket = new SocketData {
      port = 60001,
      isRunning = false,
      name = "Action"
    };

    private object serverLock = new object();

    // OPTIMIZATION: Reusable buffers
    private static readonly byte[] errorResponse = new byte[] { 1 };

    // Public and global access to the active MjSceneGenerationContext.
    public MjcfGenerationContext GenerationContext {
      get {
        if (_generationContext == null) {
          throw new InvalidOperationException(
              "This property can only be accessed from the scope of MjComponent.GenerateMjcf().");
        }
        return _generationContext;
      }
    }

    public static MjScene Instance {
      get {
        if (_instance == null) {
          var instances = FindObjectsOfType<MjScene>();
          if (instances.Length >= 1) {
            throw new InvalidOperationException(
                "A MjScene singleton is created automatically, yet multiple instances exist.");
          } else {
            GameObject go = new GameObject("MjScene");
            _instance = go.AddComponent<MjScene>();
          }
        }
        return _instance;
      }
    }

    public static bool InstanceExists { get => _instance != null; }

    public void Awake() {
      if (_instance == null) {
        _instance = this;
      } else if (_instance != this) {
        throw new InvalidOperationException(
            "MjScene is a singleton, yet multiple instances found.");
      }
      InitializeSliderController();
    }

    private static MjScene _instance = null;
    private List<MjComponent> _orderedComponents;
    public event EventHandler<MjStepArgs> postInitEvent;
    public event EventHandler<MjStepArgs> preUpdateEvent;
    public event EventHandler<MjStepArgs> ctrlCallback;
    public event EventHandler<MjStepArgs> postUpdateEvent;
    public event EventHandler<MjStepArgs> preDestroyEvent;

    private void InitializeArrowControllers() {
      ArrowControllers = new List<ArrowController>();
      for (int j = 0; j < siteCtrlIds.Length; j++) {
        var arrowGO = GameObject.Find($"1st{j+1}/Arrow_{j+1}");
        if (arrowGO == null) {
          Debug.LogWarning($"GameObject for Arrow_{j+1} not found.");
          continue;
        }
        var ctrl = arrowGO.GetComponent<ArrowController>();
        if (ctrl != null) {
          ArrowControllers.Add(ctrl);
          ctrl.SetLength(0.0f);
        } else {
          Debug.LogWarning($"ArrowController not found on Arrow_{j+1}.");
        }
      }
    }

    private IEnumerator DeferredArrowInit() {
      yield return null;
      InitializeArrowControllers();
    }

    private void InitializeSliderController() {
      sliderController = FindObjectOfType<SliderController>();
      if (sliderController != null && sliderController.speedSlider == null){
        var slider = GameObject.Find("UI/SpeedSlider")?.GetComponent<Slider>();
        if (slider == null){
          slider = GameObject.FindObjectsOfType<Slider>()
            .FirstOrDefault(s => s.gameObject.name == "SpeedSlider");
        }
        if (slider != null){
          sliderController.speedSlider = slider;
        } else {
          Debug.LogWarning("Could not auto-assign speedSlider - Slider GameObject not found.");
        }
      }
    }

    protected unsafe void Start() {
      Application.targetFrameRate = 50;  // Force 50 FPS
      Time.fixedDeltaTime = 0.02f;       // Ensure 50Hz physics

      SceneRecreationAtLateUpdateRequested = false;
      CreateScene();
      StartCoroutine(DeferredArrowInit());
      ctrlCallback += OnControlCallback;
      if (enableSocketServer) {
        NetHelpMessageLength = CalculateHelpMessageLength(Commands);
        StartSocketServer(actionSocket);
      }
    }

    protected unsafe void OnDestroy() {
      StopSocketServer(actionSocket);
      DestroyScene();
    }

    protected unsafe void FixedUpdate() {
      lock (serverSimulationLock) {
        float newViscosity;
        if (float.TryParse(viscosityTextField.text, out newViscosity)) {
          Model->opt.viscosity = 0.00009f * newViscosity;
        }
        float newTimestep;
        if (float.TryParse(timestepTextField.text, out newTimestep)) {
          Model->opt.timestep = newTimestep;
        }

        if(isSync == 1.0){
          // TODO: Change color of visual element
        }

        while (pendingSteps.Count > 0) {
          numSteps += pendingSteps.Dequeue();
        }

        // FIXED: Execute only a limited number of steps per frame
        // const int MAX_STEPS_PER_FRAME = 1;  // Process 1 step per FixedUpdate (50Hz)
        // int stepsToExecute = Math.Min(numSteps, MAX_STEPS_PER_FRAME);

        // int stepsToExecute = numSteps;
        
        // Unity's Time.fixedDeltaTime should equal Model->opt.timestep for real-time
        float physicsTimestep = (float)Model->opt.timestep;
        // Process multiple steps per frame if Unity runs faster than physics needs
        int targetStepsPerFrame = Mathf.Max(1, Mathf.RoundToInt(physicsTimestep / Time.fixedDeltaTime));
        targetStepsPerFrame = 5; // HOT FIX!
        int stepsToExecute = Math.Min(numSteps, targetStepsPerFrame);

        if (stepsToExecute > 0) {
          timer.Track(0);
          for (int i = 0; i < stepsToExecute; i++) {
            preUpdateEvent?.Invoke(this, new MjStepArgs(Model, Data));
            StepScene();
            postUpdateEvent?.Invoke(this, new MjStepArgs(Model, Data));
            numSteps--;
          }
          timer.Track(0);
        }
        simtimeTextField.text = Data->time.ToString("F4");
        realtimeTextField.text = ((float)timer.GetElapsedSeconds(0)).ToString("F4");
        distanceTextField.text = ((float)Data->qpos[0]).ToString("F4");

        if (numSteps == 0) {
          Debug.Log("SIMULATION: All steps complete, pulsing sync waiters");
          Monitor.PulseAll(serverSimulationLock);
        }
      }
    }

    public bool SceneRecreationAtLateUpdateRequested = false;

    protected unsafe void LateUpdate() {
      if (SceneRecreationAtLateUpdateRequested) {
        RecreateScene();
        SceneRecreationAtLateUpdateRequested = false;
      }
    }
    private MjcfGenerationContext _generationContext;

    public unsafe XmlDocument CreateScene(bool skipCompile=false) {
      if (_generationContext != null) {
        throw new InvalidOperationException("The scene is currently being generated on another thread.");
      }
      var hierarchyRoots = FindObjectsOfType<MjComponent>()
        .Where(component => MjHierarchyTool.FindParentComponent(component) == null)
        .Select(component => component.transform)
        .Distinct();
      _orderedComponents = new List<MjComponent>();
      foreach (var root in hierarchyRoots) {
        _orderedComponents.AddRange(MjHierarchyTool.LinearizeHierarchyBFS(root));
      }

      XmlDocument sceneMjcf = null;
      try {
        _generationContext = new MjcfGenerationContext();
        sceneMjcf = GenerateSceneMjcf(_orderedComponents);
      } catch (Exception e) {
        _generationContext = null;
        Debug.LogException(e);
        #if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;
        #else
        Application.Quit();
        #endif
        throw;
      }
      _generationContext = null;

      var settings = MjGlobalSettings.Instance;
      if (settings && !string.IsNullOrEmpty(settings.DebugFileName)) {
        SaveToFile(sceneMjcf, Path.Combine(Application.temporaryCachePath, settings.DebugFileName));
      }

      if (!skipCompile) {
        CompileScene(sceneMjcf, _orderedComponents);
      }
      postInitEvent?.Invoke(this, new MjStepArgs(Model, Data));
      return sceneMjcf;
    }

    private unsafe void CompileScene(XmlDocument mjcf, IEnumerable<MjComponent> components) {
      Model = MjEngineTool.LoadModelFromString(mjcf.OuterXml);
      if (Model == null) throw new NullReferenceException("Model loading failed.");
      Data = MujocoLib.mj_makeData(Model);
      if (Data == null) throw new NullReferenceException("mj_makeData failed.");
      foreach (var component in components) {
        component.BindToRuntime(Model, Data);
      }
    }

    public unsafe void SyncUnityToMjState() {
      foreach (var component in _orderedComponents) {
        if (component != null && component.isActiveAndEnabled) {
          component.OnSyncState(Data);
        }
      }
    }

    public unsafe void RecreateScene() {
      var joints = FindObjectsOfType<MjBaseJoint>();
      var positions = new Dictionary<MjBaseJoint, double[]>();
      var velocities = new Dictionary<MjBaseJoint, double[]>();
      foreach (var joint in joints) {
        if (joint.QposAddress > -1) {
          switch (Model->jnt_type[joint.MujocoId]) {
            default:
            case (int)MujocoLib.mjtJoint.mjJNT_HINGE:
            case (int)MujocoLib.mjtJoint.mjJNT_SLIDE:
              positions[joint] = new double[] {Data->qpos[joint.QposAddress]};
              velocities[joint] = new double[] {Data->qvel[joint.DofAddress]};
              break;
            case (int)MujocoLib.mjtJoint.mjJNT_BALL:
              positions[joint] = new double[] {
                Data->qpos[joint.QposAddress], Data->qpos[joint.QposAddress+1],
                Data->qpos[joint.QposAddress+2], Data->qpos[joint.QposAddress+3]};
              velocities[joint] = new double[] {
                Data->qvel[joint.DofAddress], Data->qvel[joint.DofAddress+1],
                Data->qvel[joint.DofAddress+2]};
              break;
            case (int)MujocoLib.mjtJoint.mjJNT_FREE:
              positions[joint] = new double[] {
                Data->qpos[joint.QposAddress], Data->qpos[joint.QposAddress+1],
                Data->qpos[joint.QposAddress+2], Data->qpos[joint.QposAddress+3],
                Data->qpos[joint.QposAddress+4], Data->qpos[joint.QposAddress+5],
                Data->qpos[joint.QposAddress+6]};
              velocities[joint] = new double[] {
                Data->qvel[joint.DofAddress], Data->qvel[joint.DofAddress+1],
                Data->qvel[joint.DofAddress+2], Data->qvel[joint.DofAddress+3],
                Data->qvel[joint.DofAddress+4], Data->qvel[joint.DofAddress+5]};
              break;
          }
        }
      }
      MujocoLib.mj_resetData(Model, Data);
      MujocoLib.mj_kinematics(Model, Data);
      SyncUnityToMjState();
      DestroyScene();
      CreateScene();
      foreach (var joint in joints) {
        try {
          var position = positions[joint];
          var velocity = velocities[joint];
          switch (Model->jnt_type[joint.MujocoId]) {
            default:
            case (int)MujocoLib.mjtJoint.mjJNT_HINGE:
            case (int)MujocoLib.mjtJoint.mjJNT_SLIDE:
              Data->qpos[joint.QposAddress] = position[0];
              Data->qvel[joint.DofAddress] = velocity[0];
              break;
            case (int)MujocoLib.mjtJoint.mjJNT_BALL:
              Data->qpos[joint.QposAddress] = position[0];
              Data->qpos[joint.QposAddress+1] = position[1];
              Data->qpos[joint.QposAddress+2] = position[2];
              Data->qpos[joint.QposAddress+3] = position[3];
              Data->qvel[joint.DofAddress] = velocity[0];
              Data->qvel[joint.DofAddress+1] = velocity[1];
              Data->qvel[joint.DofAddress+2] = velocity[2];
              break;
            case (int)MujocoLib.mjtJoint.mjJNT_FREE:
              Data->qpos[joint.QposAddress] = position[0];
              Data->qpos[joint.QposAddress+1] = position[1];
              Data->qpos[joint.QposAddress+2] = position[2];
              Data->qpos[joint.QposAddress+3] = position[3];
              Data->qpos[joint.QposAddress+4] = position[4];
              Data->qpos[joint.QposAddress+5] = position[5];
              Data->qpos[joint.QposAddress+6] = position[6];
              Data->qvel[joint.DofAddress] = velocity[0];
              Data->qvel[joint.DofAddress+1] = velocity[1];
              Data->qvel[joint.DofAddress+2] = velocity[2];
              Data->qvel[joint.DofAddress+3] = velocity[3];
              Data->qvel[joint.DofAddress+4] = velocity[4];
              Data->qvel[joint.DofAddress+5] = velocity[5];
              break;
          }
        } catch {}
      }
      MujocoLib.mj_kinematics(Model, Data);
      SyncUnityToMjState();
    }

    public unsafe void DestroyScene() {
      preDestroyEvent?.Invoke(this, new MjStepArgs(Model, Data));
      if (Model != null) {
        MujocoLib.mj_deleteModel(Model);
        Model = null;
      }
      if (Data != null) {
        MujocoLib.mj_deleteData(Data);
        Data = null;
      }
    }

    private static double[] _Qorn = new double[4] {0, 0, 0, 0};
    private static double[] _orn = new double[3] {0, 0, 0};

    [Header("Buoyancy Physics")]
    public bool enableBuoyancyPhysics = true;

    private unsafe void ApplyBuoyancyPhysics() {
        if (!enableBuoyancyPhysics || Model == null || Data == null) return;
        const float water_density = 1000.0f;
        const float gravity = 9.806f;
        const float _water_height = 0.2f;
        float waterHeight_offset = float.Parse(waterSurfaceHeight.text);
        float water_surface_height = _water_height + waterHeight_offset;
        const float height = 0.254f;
        float actual_gravity_force = Math.Abs((float)Data->qfrc_applied[2]);
        float gravity_compensation_per_corner = actual_gravity_force / 4.0f;
        const float buoyancy_coefficient = 0.2f;
        float max_additional_buoyancy_per_corner = (water_density * gravity * height * 0.01f) * buoyancy_coefficient;
        int[] siteIds = new int[] { 1, 2, 3, 4 };
        double[] myTorque = new double[3] { 0.0, 0.0, 0.0 };
        int bodyId = 1;
        for (int i = 0; i < siteIds.Length; i++) {
            float total_upward_force = gravity_compensation_per_corner;
            float site_z = (float)Data->site_xpos[3 * siteIds[i] + 2];
            float submersion_depth = water_surface_height - site_z;
            if (submersion_depth > 0.0f) {
                float effective_depth = (submersion_depth > height) ? height : submersion_depth;
                float submerged_fraction = effective_depth / height;
                float additional_buoyancy = max_additional_buoyancy_per_corner * submerged_fraction;
                total_upward_force += additional_buoyancy + 2.467f ;
            }
            double[] myForce = new double[3] { 0.0, 0.0, total_upward_force };
            double* point_on_body = Data->site_xpos + 3 * siteIds[i];
            fixed (double* f = myForce)
            fixed (double* t = myTorque) {
                MujocoLib.mj_applyFT(Model, Data, f, t, point_on_body, bodyId, Data->qfrc_applied);
            }
        }
    }

    private unsafe void OnControlCallback(object sender, MjStepArgs args) {
      lock (serverSimulationLock) {
        ApplyBuoyancyPhysics();
        if(pendingControlForces.Count > 0){
          _currentControlForces = pendingControlForces.Dequeue();
        }
        if (hasControlForces) {
          double[] myTorque = new double[3] { 0.0, 0.0, 0.0 };
          int bodyId = 1;
          for (int j = 0; j < siteCtrlIds.Length && j < _currentControlForces.Length; j++) {
            int siteId = siteCtrlIds[j];
            double* point_on_body = Data->site_xpos + 3 * siteId;
            double* siteMatrix = Data->site_xmat + 9 * siteId;
            double[] globalDirection = new double[3];
            globalDirection[0] = siteMatrix[2];
            globalDirection[1] = siteMatrix[5];
            globalDirection[2] = siteMatrix[8];
            double forceMagnitude = _currentControlForces[j];
            double[] globalForce = new double[3] {
              globalDirection[0] * forceMagnitude * float.Parse(forceMultiplier.text),
              globalDirection[1] * forceMagnitude * float.Parse(forceMultiplier.text),
              globalDirection[2] * forceMagnitude * float.Parse(forceMultiplier.text),
            };
            if(ArrowControllers.Count > j) ArrowControllers[j].SetLength(0.02f*(float)forceMagnitude);
            fixed (double* f = globalForce)
            fixed (double* t = myTorque) {
              MujocoLib.mj_applyFT(Model, Data, f, t, point_on_body, bodyId, Data->qfrc_applied);
            }
          }
        }
      }
    }

    public unsafe void StepScene() {
      if (Model == null || Data == null) throw new NullReferenceException("Mujoco runtime not created.");
      Profiler.BeginSample("MjStep");
      int nv = Model->nv;
      for (int i = 0; i < nv; i++) {
        Data->qfrc_applied[i] = 0.0;
      }
      ApplyBuoyancyPhysics();
      Profiler.BeginSample("MjStep.mj_step");
      if (ctrlCallback != null){
        MujocoLib.mj_step1(Model, Data);
        ctrlCallback?.Invoke(this, new MjStepArgs(Model, Data));
        MujocoLib.mj_step2(Model, Data);
      }
      else {
        MujocoLib.mj_step(Model, Data);
      }
      Profiler.EndSample();
      CheckForPhysicsException();
      Profiler.BeginSample("MjStep.OnSyncState");
      SyncUnityToMjState();
      Profiler.EndSample();
      Profiler.EndSample();
    }

    private unsafe void CheckForPhysicsException() {
      if (Data->warning0.number > 0) { Data->warning0.number = 0; throw new PhysicsRuntimeException("INERTIA"); }
      if (Data->warning1.number > 0) { Data->warning1.number = 0; throw new PhysicsRuntimeException("CONTACTFULL"); }
      if (Data->warning2.number > 0) { Data->warning2.number = 0; throw new PhysicsRuntimeException("CNSTRFULL"); }
      if (Data->warning3.number > 0) { Data->warning3.number = 0; throw new PhysicsRuntimeException("VGEOMFULL"); }
      if (Data->warning4.number > 0) { Data->warning4.number = 0; throw new PhysicsRuntimeException("BADQPOS"); }
      if (Data->warning5.number > 0) { Data->warning5.number = 0; throw new PhysicsRuntimeException("BADQVEL"); }
      if (Data->warning6.number > 0) { Data->warning6.number = 0; throw new PhysicsRuntimeException("BADQACC"); }
      if (Data->warning7.number > 0) { Data->warning7.number = 0; throw new PhysicsRuntimeException("BADCTRL"); }
    }

    private XmlDocument GenerateSceneMjcf(IEnumerable<MjComponent> components) {
      var doc = new XmlDocument();
      var MjRoot = (XmlElement)doc.AppendChild(doc.CreateElement("mujoco"));
      var worldMjcf = (XmlElement)MjRoot.AppendChild(doc.CreateElement("worldbody"));
      BuildHierarchicalMjcf(doc, components.Where(c => (c is MjBaseBody) || (c is MjInertial) || (c is MjBaseJoint) || (c is MjGeom) || (c is MjSite)), worldMjcf);
      MjRoot.AppendChild(GenerateMjcfSection(doc, components.Where(c => c is MjExclude), "contact"));
      MjRoot.AppendChild(GenerateMjcfSection(doc, components.Where(c => c is MjBaseTendon), "tendon"));
      MjRoot.AppendChild(GenerateMjcfSection(doc, components.Where(c => c is MjBaseConstraint), "equality"));
      MjRoot.AppendChild(GenerateMjcfSection(doc, components.Where(c => c is MjActuator).OrderBy(c => c.transform.GetSiblingIndex()), "actuator"));
      MjRoot.AppendChild(GenerateMjcfSection(doc, components.Where(c => c is MjBaseSensor).OrderBy(c => c.transform.GetSiblingIndex()), "sensor"));
      _generationContext.GenerateMjcf(MjRoot);
      return doc;
    }

    private XmlElement GenerateMjcfSection(XmlDocument doc, IEnumerable<MjComponent> components, string sectionName) {
      var section = doc.CreateElement(sectionName);
      foreach (var component in components) {
        section.AppendChild(component.GenerateMjcf(_generationContext.GenerateName(component), doc));
      }
      return section;
    }

    private void BuildHierarchicalMjcf(XmlDocument doc, IEnumerable<MjComponent> components, XmlElement worldMjcf) {
      var associations = new Dictionary<MjComponent, XmlElement>();
      foreach (var component in components) {
        associations.Add(component, component.GenerateMjcf(_generationContext.GenerateName(component), doc));
      }
      foreach (var component in components) {
        var parentComponent = MjHierarchyTool.FindParentComponent(component);
        if (parentComponent != null) {
          associations[parentComponent].AppendChild(associations[component]);
        } else {
          worldMjcf.AppendChild(associations[component]);
        }
      }
    }

    private void SaveToFile(XmlDocument document, string filePath) {
      try {
        using (var stream = File.Open(filePath, FileMode.Create))
        using (var writer = new XmlTextWriter(stream, new UTF8Encoding(false))) {
          writer.Formatting = Formatting.Indented;
          document.WriteContentTo(writer);
          Debug.Log($"MJCF saved to {filePath}");
        }
      } catch (IOException ex) {
        Debug.LogWarning("Failed to save Xml to a file: " + ex.ToString(), this);
      }
    }

    #region Socket Server Implementation

    private void StartSocketServer(SocketData socketData){
        if (!enableSocketServer) return;
        lock (serverLock) {
            if (socketData.isRunning) return;
            socketData.isRunning = true;
            socketData.thread = new Thread(() => ServerLoop(socketData));
            socketData.thread.IsBackground = true;
            socketData.thread.Start();
            Debug.Log($"{socketData.name} UDP server started on {serverIpAddress}:{socketData.port}");
        }
    }

    private void StopSocketServer(SocketData socketData){
        if (!enableSocketServer) return;
        lock (serverLock) {
            if (!socketData.isRunning) return;
            socketData.isRunning = false;
            try {
                socketData.socket?.Close();
            } catch (Exception e) {
                Debug.LogError($"Error closing {socketData.name} server socket: {e.Message}");
            }
            socketData.thread?.Join(500);
            Debug.Log($"{socketData.name} UDP server stopped");
        }
    }

    private void Server2Loop(SocketData socketData) {
        try {
            socketData.socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socketData.socket.Bind(new IPEndPoint(IPAddress.Parse(serverIpAddress), socketData.port));
            Debug.Log($"{socketData.name} UDP server listening for datagrams.");

            byte[] receiveBuffer = new byte[1024];
            EndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

            while (socketData.isRunning) {
                try {
                    if (socketData.socket.Available == 0) {
                        Thread.Sleep(1);
                        continue;
                    }
                    int bytesRead = socketData.socket.ReceiveFrom(receiveBuffer, ref remoteEndPoint);
                    if (bytesRead > 0) {
                        ThreadPool.QueueUserWorkItem(state => {
                            var tuple = (Tuple<byte[], int, EndPoint>)state;
                            ProcessUdpRequest(tuple.Item1, tuple.Item2, tuple.Item3, socketData.socket);
                        }, Tuple.Create(receiveBuffer, bytesRead, remoteEndPoint));
                    }
                } catch (SocketException se) {
                    if (socketData.isRunning) Debug.LogWarning($"{socketData.name} socket error: {se.SocketErrorCode}");
                }
            }
        } catch (Exception e) {
            if (socketData.isRunning) Debug.LogError($"{socketData.name} server loop error: {e.Message}");
        } finally {
            socketData.socket?.Close();
        }
    }

    private void ServerLoop(SocketData socketData) {
      try {
        socketData.socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        socketData.socket.Bind(new IPEndPoint(IPAddress.Parse(serverIpAddress), socketData.port));
        Debug.Log($"{socketData.name} UDP server listening for datagrams.");

        // This buffer is now only a temporary holding place for each read.
        byte[] receiveBuffer = new byte[1024]; 
        EndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

        while (socketData.isRunning) {
          try {
            if (socketData.socket.Available == 0) {
              Thread.Sleep(1);
              continue;
            }
            int bytesRead = socketData.socket.ReceiveFrom(receiveBuffer, ref remoteEndPoint);

            if (bytesRead > 0) {
              // * FIX: Create a copy of the received data for this specific request.
              // This ensures that each thread in the ThreadPool gets its own isolated
              // data and prevents the receiveBuffer from being overwritten by the next packet.
              byte[] requestDataCopy = new byte[bytesRead];
              Buffer.BlockCopy(receiveBuffer, 0, requestDataCopy, 0, bytesRead);

              ThreadPool.QueueUserWorkItem(state => {
                  // The state now contains the isolated copy of the data.
                  var tuple = (Tuple<byte[], int, EndPoint>)state;
                  ProcessUdpRequest(tuple.Item1, tuple.Item2, tuple.Item3, socketData.socket);
                  }, Tuple.Create(requestDataCopy, bytesRead, remoteEndPoint));
            }
          } catch (SocketException se) {
            if (socketData.isRunning) Debug.LogWarning($"{socketData.name} socket error: {se.SocketErrorCode}");
          }
        }
      } catch (Exception e) {
        if (socketData.isRunning) Debug.LogError($"{socketData.name} server loop error: {e.Message}");
      } finally {
        socketData.socket?.Close();
      }
    }

    private unsafe void ProcessUdpRequest(byte[] requestData, int length, EndPoint remoteEndPoint, Socket serverSocket) {
        try {
            if (length < 4) return;
            float commandFloat = BitConverter.ToSingle(requestData, 0);
            int command = (int)commandFloat;
            
            float[] floatsReceived = new float[length / 4];
            Buffer.BlockCopy(requestData, 0, floatsReceived, 0, length);
            
            ProcessCommandOptimized(command, floatsReceived, remoteEndPoint, serverSocket);

        } catch (Exception e) {
            Debug.LogError($"Error processing UDP request: {e.Message}");
            try {
                serverSocket.SendTo(errorResponse, remoteEndPoint);
            } catch (Exception sendEx) {
                Debug.LogError($"Failed to send error response: {sendEx.Message}");
            }
        }
    }

    private unsafe void ProcessCommandOptimized(int command, float[] floatsReceived, EndPoint remoteEndPoint, Socket serverSocket) {
      byte[] responsePayload = null;

      switch ((MsgHeader)command) {
        case MsgHeader.HEARTBEAT:
          Debug.Log("HEARTBEAT received!");
          serverSocket.SendTo(new byte[] { 0 }, remoteEndPoint);
          break;

        case MsgHeader.GET_MODEL_INFO:
          Debug.Log("GET_MODEL_INFO received!");
          float[] model_info = new float[6] {
            0, (float)Model->nq, (float)Model->nv,
            (float)Model->na, (float)Model->nu, (float)Model->nbody
          };
          lock (serverSimulationLock) {
            model_info[0] = (float)Data->time;
          }
          responsePayload = new byte[24];
          Buffer.BlockCopy(model_info, 0, responsePayload, 0, 24);
          serverSocket.SendTo(responsePayload, remoteEndPoint);
          break;

        case MsgHeader.GET_SENSORDATA:
          Debug.Log("GET_SENSORDATA received!");
          // Format: [current_orientaiton quaternion, current_time, depth, 4xfloats(for bounding box)]
          int nsensors = (int)Model->nsensordata;
          int len = nsensors + 1 + 1 + 4;
          float[] sensorData = new float[len];
          lock(serverSimulationLock){ // These need the lock since it needs to measure time
            double* sourcePtr = Data->sensordata;
            for (int i = 0; i < len - 5; i++) {
              sensorData[i] = (float)sourcePtr[i];
            }
            sensorData[nsensors] = (float)Data->time;
            sensorData[nsensors + 1] = (float)Data->qpos[2];

            // Target Submarine BBox
            // Format: { bbox.x, bbox.y, bbox.width, bbox.height };
            float[] bboxData = ThreadSafeBoundingBoxProvider.Instance.GetLatestBoundingBoxData();
            sensorData[nsensors + 2] = bboxData[0]; // x
            sensorData[nsensors + 3] = bboxData[1]; // y
            sensorData[nsensors + 4] = bboxData[2]; // width
            sensorData[nsensors + 5] = bboxData[3]; // height
          }
          responsePayload = new byte[len * sizeof(float)];
          Buffer.BlockCopy(sensorData, 0, responsePayload, 0, responsePayload.Length);
          serverSocket.SendTo(responsePayload, remoteEndPoint);
          break;

        case MsgHeader.APPLY_CTRL:
          lock (serverSimulationLock) {
            // The client sends 8 floats total (header + 7 args)
            // Server sends: [status_byte(x1), sensorData (x nsensorsdata), time(x1), bbox(x4) ]
            if (floatsReceived.Length < 8) {
              Debug.Log("Error: APPLY_CTRL expects 8 float values (header, nsteps, sync_flag, 6x thrusters).");
              break;
            }

            // FIX: Indices are shifted by 1 because the header is at index 0.
            int nstepsToSim = (int)floatsReceived[1];
            isSync = (int)floatsReceived[2];

            if (isSync != 0 && isSync != 1) { 
              Debug.Log($"Error in Sync/Async!: Invalid number received floatsReceived[2]={(float)isSync}. Expected 0.0 or 1.0.");
              isSync = 1; // Default behaviour : Sync
            }

            float[] forces = new float[6];
            // FIX: Force data starts at index 3.
            Array.Copy(floatsReceived, 3, forces, 0, 6);
            hasControlForces = true;
            pendingControlForces.Enqueue(forces);
            pendingSteps.Enqueue(nstepsToSim);

            Debug.Log($"APPLY_CTRL received: nsteps={nstepsToSim}, sync_flag={isSync}");

            if(isSync == 1){
              while (numSteps > 0 || pendingSteps.Count > 0) {
                Monitor.Wait(serverSimulationLock);
              }
            }
            // Target Submarine BBox
            // Format: { bbox.x, bbox.y, bbox.width, bbox.height };
            float[] bboxData = ThreadSafeBoundingBoxProvider.Instance.GetLatestBoundingBoxData();
            responsePayload = CreateSensorAndBboxPayload(bboxData);
          }
          if (responsePayload != null) serverSocket.SendTo(responsePayload, remoteEndPoint);
          break;

        case MsgHeader.STEP_SIM:
          lock (serverSimulationLock) {
            // Client sends 3 floats (header, nsteps, sync_flag)
            if (floatsReceived.Length < 3) {
              Debug.Log("Error: STEP_SIM expects 3 float values (header, nsteps, sync_flag).");
              break; 
            }

            // FIX: Indices are shifted by 1.
            int nsteps = (int)floatsReceived[1];
            isSync = (int)floatsReceived[2];

            pendingSteps.Enqueue(nsteps);
            Debug.Log($"STEP_SIM received: nsteps={nsteps}, sync_flag={isSync}");

            if (isSync == 1) {
              while (numSteps > 0 || pendingSteps.Count > 0) {
                Monitor.Wait(serverSimulationLock);
              }
            }
            // responsePayload = GetSensorDataPayloadWithStatus();
            float[] bboxData = ThreadSafeBoundingBoxProvider.Instance.GetLatestBoundingBoxData();
            responsePayload = CreateSensorAndBboxPayload(bboxData);
          }
          if (responsePayload != null) serverSocket.SendTo(responsePayload, remoteEndPoint);
          break;

        case MsgHeader.RESET:
          lock (serverSimulationLock) {
            // Client sends 16 floats (header , nsteps , isSync , Px,Py,Pz, Ow,Ox,Oy,Oz, LVx, LVy, Lvz, AVx, AVy, AVz )
            if (floatsReceived.Length < 16) {
              Debug.Log("Error: RESET expects 16 float values.");
              break; 
            }
            ZeroOutAppliedForces();
            pendingControlForces.Clear();
            Array.Clear(_currentControlForces, 0, _currentControlForces.Length);
            hasControlForces = false;
            MujocoLib.mj_resetData(Model, Data);

            // FIX: Pass the correctly indexed array to the helper function.
            ApplyQposQvelToSim(floatsReceived);
            MujocoLib.mj_kinematics(Model, Data);

            // FIX: Indices are shifted by 1.
            int nsteps = (int)floatsReceived[1];
            isSync = (int)floatsReceived[2];

            pendingSteps.Enqueue(nsteps);
            if (isSync != 0 && isSync !=1) { 
              Debug.Log($"Error in Sync/Async!: Invalid number received floatsReceived[2]={(float)isSync}. Expected 0.0 or 1.0.");
              isSync = 1;
            }
            Debug.Log($"RESET received: nsteps={nsteps}, sync_flag={isSync}");

            if (isSync == 1) {
              while (numSteps > 0 || pendingSteps.Count > 0) {
                Monitor.Wait(serverSimulationLock);
              }
            }
            // responsePayload = GetSensorDataPayloadWithStatus();
            float[] bboxData = ThreadSafeBoundingBoxProvider.Instance.GetLatestBoundingBoxData();
            responsePayload = CreateSensorAndBboxPayload(bboxData);
          }
          if (responsePayload != null) serverSocket.SendTo(responsePayload, remoteEndPoint);
          break;

        case MsgHeader.GET_RGB_IMAGE:
        case MsgHeader.GET_MASKED_IMAGE:
          serverSocket.SendTo(errorResponse, remoteEndPoint);
          break;

        default:
          string helpMessage = GenerateHelpMessage();
          byte[] helpBytes = Encoding.UTF8.GetBytes(helpMessage);
          serverSocket.SendTo(helpBytes, remoteEndPoint);
          break;
      }
    }

    private unsafe bool ApplyQposQvelToSim(float[] floatsReceived){
      // FIX: The data we care about now starts at index 3 (after header, nsteps, sync_flag).
      int offset = 3; 
      Data->qpos[0] = (double)floatsReceived[offset + 0];
      Data->qpos[1] = (double)floatsReceived[offset + 1];
      Data->qpos[2] = (double)floatsReceived[offset + 2];
      Data->qpos[3] = (double)floatsReceived[offset + 3];
      Data->qpos[4] = (double)floatsReceived[offset + 4];
      Data->qpos[5] = (double)floatsReceived[offset + 5];
      Data->qpos[6] = (double)floatsReceived[offset + 6];

      Data->qvel[0] = (double)floatsReceived[offset + 7];
      Data->qvel[1] = (double)floatsReceived[offset + 8];
      Data->qvel[2] = (double)floatsReceived[offset + 9];
      Data->qvel[3] = (double)floatsReceived[offset + 10];
      Data->qvel[4] = (double)floatsReceived[offset + 11];
      Data->qvel[5] = (double)floatsReceived[offset + 12];
      return true;
    }

    private unsafe byte[] CreateSensorAndBboxPayload(float[] bboxData){
      // --- 1. Calculate the total size needed ---
      int sensorFloatCount = (int)Model->nsensordata + 1; // +1 for time
      int depthFloatCount = 1;
      int bboxFloatCount = 4;
      int totalPayloadSize = 1 + (sensorFloatCount * sizeof(float)) + (depthFloatCount * sizeof(float)) + (bboxFloatCount * sizeof(float));

      byte[] responsePayload = new byte[totalPayloadSize];
      responsePayload[0] = 0x00; // Success byte

      // Copy sensor data and time
      float[] tempSensorFloats = new float[sensorFloatCount];
      double* sourcePtr = Data->sensordata;
      for (int i = 0; i < sensorFloatCount - 1; i++) {
        tempSensorFloats[i] = (float)sourcePtr[i];
      }
      tempSensorFloats[sensorFloatCount - 1] = (float)Data->time;
      Buffer.BlockCopy(tempSensorFloats, 0, responsePayload, 1, sensorFloatCount * sizeof(float));

      // --- ADD MISSING DEPTH VALUE ---
      int depthOffset = 1 + (sensorFloatCount * sizeof(float));
      float depthValue = (float)Data->qpos[2]; // Depth of submarine
      Buffer.BlockCopy(BitConverter.GetBytes(depthValue), 0, responsePayload, depthOffset, sizeof(float));

      // Copy bounding box data
      int bboxOffset = depthOffset + (depthFloatCount * sizeof(float));
      Buffer.BlockCopy(bboxData, 0, responsePayload, bboxOffset, bboxFloatCount * sizeof(float));

      return responsePayload;
    }

    private unsafe byte[] GetSensorDataPayloadWithStatus() {
        int len = (int)Model->nsensordata + 1;
        float[] sensorData = new float[len];
        byte[] responsePayload = new byte[1 + len * sizeof(float)];
        
        responsePayload[0] = 0x00; // Success byte
        
        double* sourcePtr = Data->sensordata;
        for (int i = 0; i < len - 1; i++) {
            sensorData[i] = (float)sourcePtr[i];
        }
        sensorData[len - 1] = (float)Data->time;
        
        Buffer.BlockCopy(sensorData, 0, responsePayload, 1, len * sizeof(float));
        return responsePayload;
    }

    private unsafe void ZeroOutAppliedForces(){
      if (Model == null || Data == null) return;
      int nv = (int)Model->nv;
      for (int i = 0; i < nv; i++){
        Data->qfrc_applied[i] = 0.0;
      }
    }
    
    private string GenerateHelpMessage() {
      var sb = new StringBuilder(NetHelpMessageLength + 1);
      sb.AppendLine("=== AVAILABLE COMMANDS (UDP) ===");
      sb.AppendLine("Response Codes: 0=SUCCESS, 1=ERROR");
      sb.AppendLine();
      foreach (var cmd in Commands) {
        sb.AppendLine($"[{ (int)cmd.MsgHeader }.0] {cmd.Name}:\n  {cmd.HelpString}");
      }
      sb.AppendLine("=== END COMMANDS ===");
      return sb.ToString();
    }

    private static int CalculateHelpMessageLength(CommandInfo[] CommandsStruct) {
        string header = "=== AVAILABLE COMMANDS (UDP) ===" + Environment.NewLine +
                        "Response Codes: 0=SUCCESS, 1=ERROR" + Environment.NewLine + Environment.NewLine;
        string footer = "=== END COMMANDS ===" + Environment.NewLine;
        int totalLength = Encoding.UTF8.GetByteCount(header) + Encoding.UTF8.GetByteCount(footer);
        foreach (var cmd in CommandsStruct) {
            string commandLine = $"[{(int)cmd.MsgHeader}.0] {cmd.Name}:\n  {cmd.HelpString}";
            totalLength += Encoding.UTF8.GetByteCount(commandLine);
        }
        return totalLength;
    }
    #endregion
  }

  public class MjStepArgs : EventArgs {
    public unsafe MjStepArgs(MujocoLib.mjModel_* model, MujocoLib.mjData_* data){
      this.model = model;
      this.data = data;
    }
    public readonly unsafe MujocoLib.mjModel_* model;
    public readonly unsafe MujocoLib.mjData_* data;
  }
}
