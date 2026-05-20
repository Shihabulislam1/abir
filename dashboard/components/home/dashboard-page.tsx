"use client";

import * as React from "react";
import * as ROSLIB from "roslib";
import { 
  Play, 
  Square, 
  Settings, 
  Activity, 
  Wifi, 
  WifiOff, 
  Camera, 
  Sliders, 
  Keyboard, 
  AlertTriangle, 
  Shield, 
  Compass,
  ArrowUp,
  ArrowLeft,
  ArrowDown,
  ArrowRight
} from "lucide-react";
import { Button } from "@/components/ui/button";
import { Card, CardHeader, CardTitle, CardContent } from "@/components/ui/card";
import { Slider } from "@/components/ui/slider";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { LidarVisualization } from "./lidar-visualization";

type ConnStatus = "disconnected" | "connecting" | "connected" | "error";

interface RobotStatus {
  enabled: boolean;
  mode: "auto" | "manual";
  state: number;
  lane_error: number;
  obstacle_ahead: boolean;
  side_clear: boolean;
  kp: number;
  kd: number;
  base_speed: number;
  nudge_duration: number;
}

export function DashboardPage() {
  const [wsUri, setWsUri] = React.useState<string>("ws://localhost:9090");
  const [ros, setRos] = React.useState<ROSLIB.Ros | null>(null);
  const [connStatus, setConnStatus] = React.useState<ConnStatus>("disconnected");
  
  // Local copies of tunable parameters (for slider UI)
  const [kp, setKp] = React.useState<number>(0.005);
  const [kd, setKd] = React.useState<number>(0.001);
  const [baseSpeed, setBaseSpeed] = React.useState<number>(0.3);
  const [nudgeDuration, setNudgeDuration] = React.useState<number>(1.2);
  const [frontDangerZone, setFrontDangerZone] = React.useState<number>(0.6);
  const [sideSafeZone, setSideSafeZone] = React.useState<number>(0.8);
  const [lidarOffsetDeg, setLidarOffsetDeg] = React.useState<number>(0.0);

  // Live telemetry received from /robot_status
  const [telemetry, setTelemetry] = React.useState<RobotStatus>({
    enabled: true,
    mode: "auto",
    state: 1,
    lane_error: 0.0,
    obstacle_ahead: false,
    side_clear: true,
    kp: 0.005,
    kd: 0.001,
    base_speed: 0.3,
    nudge_duration: 1.2,
  });

  // Track pressed keys for WASD visualization
  const [pressedKeys, setPressedKeys] = React.useState<Set<string>>(new Set());

  // Image feed state
  const imgRef = React.useRef<HTMLImageElement>(null);

  // Handle connection
  const connectToRos = React.useCallback((uri: string) => {
    if (ros) {
      try { ros.close(); } catch(e) {}
    }
    
    setConnStatus("connecting");
    const r = new ROSLIB.Ros({ url: uri });
    
    r.on("connection", () => {
      setRos(r);
      setConnStatus("connected");
    });
    
    r.on("error", () => {
      setConnStatus("error");
    });
    
    r.on("close", () => {
      setRos(null);
      setConnStatus("disconnected");
    });
  }, [ros]);

  // Connect automatically on mount
  React.useEffect(() => {
    connectToRos(wsUri);
    return () => {
      if (ros) ros.close();
    };
  }, []);

  // Set parameter helper
  const setRosParameter = React.useCallback((nodeName: string, paramName: string, paramType: number, value: any) => {
    if (!ros || connStatus !== "connected") return;
    
    const client = new ROSLIB.Service({
      ros: ros,
      name: `/${nodeName}/set_parameters`,
      serviceType: "rcl_interfaces/srv/SetParameters"
    });

    const valueObj: any = { type: paramType };
    if (paramType === 1) valueObj.bool_value = !!value;
    else if (paramType === 2) valueObj.integer_value = parseInt(value);
    else if (paramType === 3) valueObj.double_value = parseFloat(value);
    else if (paramType === 4) valueObj.string_value = String(value);

    const request = {
      parameters: [
        {
          name: paramName,
          value: valueObj
        }
      ]
    };

    client.callService(request, 
      (res: any) => console.log(`Param ${paramName} set:`, res),
      (err: any) => console.error(`Param ${paramName} set error:`, err)
    );
  }, [ros, connStatus]);

  // Subscribe to telemetry and video
  React.useEffect(() => {
    if (!ros || connStatus !== "connected") return;

    // Telemetry subscription
    const statusSub = new ROSLIB.Topic({
      ros,
      name: "/robot_status",
      messageType: "std_msgs/msg/String"
    });

    statusSub.subscribe((msg: any) => {
      try {
        const data: RobotStatus = JSON.parse(msg.data);
        setTelemetry(data);
        // Sync local parameters if they differ, so sliders load initial values from ROS node
        setKp(data.kp);
        setKd(data.kd);
        setBaseSpeed(data.base_speed);
        setNudgeDuration(data.nudge_duration);
      } catch (e) {
        console.error("Failed to parse telemetry JSON:", e);
      }
    });

    // Compressed Image subscription
    const imgSub = new ROSLIB.Topic({
      ros,
      name: "/camera/debug_image/compressed",
      messageType: "sensor_msgs/msg/CompressedImage",
      throttle_rate: 66, // Limit to ~15 FPS to save bandwidth
      queue_size: 1
    });

    imgSub.subscribe((msg: any) => {
      if (imgRef.current) {
        imgRef.current.src = `data:image/jpeg;base64,${msg.data}`;
      }
    });

    return () => {
      statusSub.unsubscribe();
      imgSub.unsubscribe();
    };
  }, [ros, connStatus]);

  // Keyboard teleoperation logic
  React.useEffect(() => {
    if (telemetry.mode !== "manual" || !ros || connStatus !== "connected") {
      setPressedKeys(new Set());
      return;
    }

    const cmdTopic = new ROSLIB.Topic({
      ros,
      name: "/cmd_vel_teleop",
      messageType: "geometry_msgs/msg/Twist"
    });

    const activeKeys = new Set<string>();

    const handleKeyDown = (e: KeyboardEvent) => {
      if (["INPUT", "TEXTAREA"].includes((e.target as HTMLElement).tagName)) return;
      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d"].includes(key)) {
        activeKeys.add(key);
        setPressedKeys(new Set(activeKeys));
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d"].includes(key)) {
        activeKeys.delete(key);
        setPressedKeys(new Set(activeKeys));
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    // Send Twist commands at 10 Hz
    const interval = setInterval(() => {
      let linear = 0.0;
      let angular = 0.0;

      if (activeKeys.has("w")) linear += baseSpeed;
      if (activeKeys.has("s")) linear -= baseSpeed;
      if (activeKeys.has("a")) angular += 0.6; // steer left
      if (activeKeys.has("d")) angular -= 0.6; // steer right

      const twist = {
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular }
      };
      cmdTopic.publish(twist);
    }, 100);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      clearInterval(interval);
    };
  }, [telemetry.mode, ros, connStatus, baseSpeed]);

  const getStateLabel = (stateNum: number) => {
    switch (stateNum) {
      case 1: return "Follow Right Lane";
      case 2: return "Nudging Left";
      case 3: return "Follow Left Lane";
      case 4: return "Nudging Right";
      default: return `Unknown (${stateNum})`;
    }
  };

  return (
    <div className="min-h-screen bg-black text-zinc-100 font-sans p-6 selection:bg-cyan-500/30">
      {/* Title Header */}
      <header className="mb-6 flex flex-col md:flex-row md:items-center md:justify-between border-b border-zinc-800 pb-4">
        <div>
          <h1 className="text-3xl font-extrabold tracking-tight bg-clip-text text-transparent bg-gradient-to-r from-cyan-400 via-sky-400 to-indigo-400">
            Autonomous Lane-Tracking AV Panel
          </h1>
          <p className="text-sm text-zinc-400 font-mono mt-1">
            ROSLIB.js WebSocket • Lidar Integration • PID Lane Tracking
          </p>
        </div>

        {/* Connection Widget */}
        <div className="flex items-center gap-3 mt-4 md:mt-0 bg-zinc-900/40 p-2 rounded-xl border border-zinc-800/80 backdrop-blur-sm">
          <Input
            value={wsUri}
            onChange={(e) => setWsUri(e.target.value)}
            className="h-8 w-44 bg-zinc-950 border-zinc-800 text-xs font-mono text-zinc-300"
            placeholder="ws://localhost:9090"
          />
          <Button
            size="sm"
            onClick={() => connectToRos(wsUri)}
            className="h-8 text-xs bg-cyan-600 hover:bg-cyan-500 text-white font-medium shadow-cyan-950/20"
          >
            Connect
          </Button>
          <div className="flex items-center gap-1.5 px-2">
            {connStatus === "connected" ? (
              <Wifi className="h-4.5 w-4.5 text-emerald-400" />
            ) : (
              <WifiOff className="h-4.5 w-4.5 text-zinc-500" />
            )}
            <Badge
              className={`text-[10px] tracking-wider px-2 py-0.5 font-bold ${
                connStatus === "connected"
                  ? "bg-emerald-500/10 text-emerald-400 border-emerald-500/20"
                  : connStatus === "connecting"
                  ? "bg-yellow-500/10 text-yellow-400 border-yellow-500/20 animate-pulse"
                  : "bg-red-500/10 text-red-400 border-red-500/20"
              }`}
              variant="outline"
            >
              {connStatus.toUpperCase()}
            </Badge>
          </div>
        </div>
      </header>

      {/* Main Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-6">
        
        {/* Left Column (Estop, Manual control & Teleop visualization) */}
        <div className="lg:col-span-3 flex flex-col gap-6">
          
          {/* Safety & Mode Controls */}
          <Card className="bg-zinc-950 border-zinc-900 shadow-xl overflow-hidden relative">
            <div className="absolute top-0 left-0 w-full h-[3px] bg-gradient-to-r from-red-500 to-indigo-500" />
            <CardHeader className="pb-3 pt-5">
              <CardTitle className="text-zinc-200 text-md font-semibold flex items-center gap-2">
                <Shield className="h-4.5 w-4.5 text-cyan-400" /> System Safety & Modes
              </CardTitle>
            </CardHeader>
            <CardContent className="flex flex-col gap-4">
              
              {/* Massive Estop Button */}
              <button
                onClick={() => {
                  const targetEnabled = !telemetry.enabled;
                  setRosParameter("brain_node", "enabled", 1, targetEnabled);
                }}
                className={`w-full py-5 rounded-2xl border transition-all duration-300 flex flex-col items-center justify-center gap-1.5 relative overflow-hidden ${
                  telemetry.enabled 
                    ? "bg-red-950/20 hover:bg-red-950/40 text-red-400 border-red-900/60 active:scale-[0.98] shadow-lg shadow-red-950/20"
                    : "bg-emerald-950/20 hover:bg-emerald-950/40 text-emerald-400 border-emerald-900/60 active:scale-[0.98] shadow-lg shadow-emerald-950/20 animate-pulse"
                }`}
              >
                {telemetry.enabled ? (
                  <>
                    <Square className="h-7 w-7 fill-red-400" />
                    <span className="text-sm font-bold tracking-wider">E-STOP ENGINE</span>
                  </>
                ) : (
                  <>
                    <Play className="h-7 w-7 fill-emerald-400 text-emerald-400" />
                    <span className="text-sm font-bold tracking-wider">RE-ARM / START</span>
                  </>
                )}
              </button>

              {/* Mode Selector Switch */}
              <div className="p-3 bg-zinc-900/40 rounded-xl border border-zinc-900 flex items-center justify-between">
                <div>
                  <div className="text-xs font-semibold text-zinc-300">Operation Mode</div>
                  <div className="text-[10px] text-zinc-500 font-mono mt-0.5">
                    {telemetry.mode === "auto" ? "Autonomous Lane Following" : "WASD Keyboard Drive"}
                  </div>
                </div>
                <div className="flex items-center gap-2">
                  <span className={`text-xs font-bold ${telemetry.mode === "manual" ? "text-cyan-400" : "text-zinc-500"}`}>MANUAL</span>
                  <button
                    onClick={() => {
                      const nextMode = telemetry.mode === "auto" ? "manual" : "auto";
                      setRosParameter("brain_node", "mode", 4, nextMode);
                    }}
                    className={`relative w-12 h-6 rounded-full transition-colors duration-300 ${
                      telemetry.mode === "auto" ? "bg-cyan-500" : "bg-zinc-800"
                    }`}
                  >
                    <span 
                      className={`absolute top-1 left-1 w-4 h-4 rounded-full bg-white transition-transform duration-300 ${
                        telemetry.mode === "auto" ? "transform translate-x-6" : ""
                      }`}
                    />
                  </button>
                  <span className={`text-xs font-bold ${telemetry.mode === "auto" ? "text-cyan-400" : "text-zinc-500"}`}>AUTO</span>
                </div>
              </div>
            </CardContent>
          </Card>

          {/* Teleop WASD Key visualizer */}
          <Card className="bg-zinc-950 border-zinc-900 shadow-xl overflow-hidden flex-1 min-h-[220px]">
            <CardHeader className="pb-2 pt-4">
              <CardTitle className="text-zinc-200 text-md font-semibold flex items-center gap-2">
                <Keyboard className="h-4.5 w-4.5 text-cyan-400" /> Keyboard Teleop
              </CardTitle>
            </CardHeader>
            <CardContent className="flex flex-col items-center justify-center h-[calc(100%-60px)] pb-4">
              <p className="text-[11px] text-zinc-500 font-mono mb-4 text-center">
                {telemetry.mode === "manual" 
                  ? "Click dashboard & hold WASD to steer" 
                  : "Switch to MANUAL mode to enable keys"}
              </p>
              
              {/* WASD Layout */}
              <div className="flex flex-col items-center gap-3">
                {/* W Key */}
                <div 
                  className={`w-14 h-14 rounded-xl flex items-center justify-center border font-bold text-lg font-mono transition-all duration-150 shadow-inner ${
                    pressedKeys.has("w") 
                      ? "bg-cyan-500/20 text-cyan-400 border-cyan-400 shadow-cyan-500/10 scale-95" 
                      : "bg-zinc-900/50 text-zinc-400 border-zinc-800"
                  }`}
                >
                  <ArrowUp className="h-5 w-5" />
                </div>
                {/* ASD Keys Row */}
                <div className="flex gap-3">
                  <div 
                    className={`w-14 h-14 rounded-xl flex items-center justify-center border font-bold text-lg font-mono transition-all duration-150 shadow-inner ${
                      pressedKeys.has("a") 
                        ? "bg-cyan-500/20 text-cyan-400 border-cyan-400 shadow-cyan-500/10 scale-95" 
                        : "bg-zinc-900/50 text-zinc-400 border-zinc-800"
                    }`}
                  >
                    <ArrowLeft className="h-5 w-5" />
                  </div>
                  <div 
                    className={`w-14 h-14 rounded-xl flex items-center justify-center border font-bold text-lg font-mono transition-all duration-150 shadow-inner ${
                      pressedKeys.has("s") 
                        ? "bg-cyan-500/20 text-cyan-400 border-cyan-400 shadow-cyan-500/10 scale-95" 
                        : "bg-zinc-900/50 text-zinc-400 border-zinc-800"
                    }`}
                  >
                    <ArrowDown className="h-5 w-5" />
                  </div>
                  <div 
                    className={`w-14 h-14 rounded-xl flex items-center justify-center border font-bold text-lg font-mono transition-all duration-150 shadow-inner ${
                      pressedKeys.has("d") 
                        ? "bg-cyan-500/20 text-cyan-400 border-cyan-400 shadow-cyan-500/10 scale-95" 
                        : "bg-zinc-900/50 text-zinc-400 border-zinc-800"
                    }`}
                  >
                    <ArrowRight className="h-5 w-5" />
                  </div>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Middle Column (Camera and Lidar visualizers) */}
        <div className="lg:col-span-5 flex flex-col gap-6">
          
          {/* Camera Debug Feed */}
          <Card className="bg-zinc-950 border-zinc-900 shadow-xl overflow-hidden flex flex-col h-[340px]">
            <CardHeader className="pb-2 pt-4 px-4 shrink-0 flex flex-row items-center justify-between border-b border-zinc-900">
              <CardTitle className="text-zinc-200 text-sm font-semibold flex items-center gap-2">
                <Camera className="h-4.5 w-4.5 text-cyan-400" /> OpenCV Drivable HUD
              </CardTitle>
              <span className="text-[10px] text-zinc-500 font-mono">/camera/debug_image/compressed</span>
            </CardHeader>
            <CardContent className="flex-1 p-0 bg-zinc-950 relative flex items-center justify-center overflow-hidden">
              <img
                ref={imgRef}
                alt="Camera debug feed"
                className="w-full h-full object-contain"
              />
              {connStatus !== "connected" && (
                <div className="absolute inset-0 bg-zinc-950/80 flex flex-col items-center justify-center text-zinc-500">
                  <Camera className="h-10 w-10 mb-2 opacity-25" />
                  <p className="text-xs">Awaiting camera websocket stream...</p>
                </div>
              )}
            </CardContent>
          </Card>

          {/* Lidar Sweep Visualization */}
          <Card className="bg-zinc-950 border-zinc-900 shadow-xl overflow-hidden h-[340px] flex flex-col">
            <div className="flex-1 min-h-0">
              <LidarVisualization ros={ros} />
            </div>
          </Card>
        </div>

        {/* Right Column (Tuning Sliders & Live Telemetry metrics) */}
        <div className="lg:col-span-4 flex flex-col gap-6">
          
          {/* Live Telemetry Status Details */}
          <Card className="bg-zinc-950 border-zinc-900 shadow-xl overflow-hidden relative">
            <div className="absolute top-0 left-0 w-full h-[3px] bg-gradient-to-r from-cyan-500 to-indigo-500" />
            <CardHeader className="pb-2 pt-4">
              <CardTitle className="text-zinc-200 text-md font-semibold flex items-center gap-2">
                <Activity className="h-4.5 w-4.5 text-cyan-400" /> Live Vehicle Status
              </CardTitle>
            </CardHeader>
            <CardContent className="grid grid-cols-2 gap-4 pb-4">
              
              <div className="p-3 bg-zinc-900/40 rounded-xl border border-zinc-900/80">
                <div className="text-[10px] text-zinc-400 font-bold uppercase tracking-wider font-mono">AV Mode</div>
                <div className="text-sm font-bold text-zinc-200 mt-1 capitalize">{telemetry.mode}</div>
              </div>

              <div className="p-3 bg-zinc-900/40 rounded-xl border border-zinc-900/80">
                <div className="text-[10px] text-zinc-400 font-bold uppercase tracking-wider font-mono">State Machine</div>
                <div className="text-xs font-bold text-zinc-200 mt-1.5 truncate">{getStateLabel(telemetry.state)}</div>
              </div>

              <div className={`p-3 rounded-xl border transition-all duration-300 ${
                telemetry.obstacle_ahead 
                  ? "bg-red-500/10 border-red-500/20 text-red-400" 
                  : "bg-zinc-900/40 border-zinc-900 text-zinc-400"
              }`}>
                <div className="text-[10px] font-bold uppercase tracking-wider font-mono flex items-center gap-1.5">
                  <AlertTriangle className="h-3.5 w-3.5" /> Front Obstacle
                </div>
                <div className="text-xs font-bold mt-1.5">
                  {telemetry.obstacle_ahead ? "DANGER / DETECTED" : "CLEAR"}
                </div>
              </div>

              <div className={`p-3 rounded-xl border transition-all duration-300 ${
                !telemetry.side_clear 
                  ? "bg-yellow-500/10 border-yellow-500/20 text-yellow-400" 
                  : "bg-emerald-500/10 border-emerald-500/20 text-emerald-400"
              }`}>
                <div className="text-[10px] font-bold uppercase tracking-wider font-mono flex items-center gap-1.5">
                  <Compass className="h-3.5 w-3.5" /> Side Corridor
                </div>
                <div className="text-xs font-bold mt-1.5">
                  {telemetry.side_clear ? "CLEAR / SAFE" : "BLOCKED"}
                </div>
              </div>

              <div className="col-span-2 p-3 bg-zinc-900/40 rounded-xl border border-zinc-900/80">
                <div className="text-[10px] text-zinc-400 font-bold uppercase tracking-wider font-mono">Calculated Steering Lane Error</div>
                <div className="flex items-center justify-between mt-1">
                  <div className="text-xl font-extrabold text-cyan-400 font-mono">
                    {telemetry.lane_error.toFixed(2)} <span className="text-[10px] text-zinc-500 font-normal">px</span>
                  </div>
                  {/* Small visual error bar */}
                  <div className="w-1/2 h-2.5 bg-zinc-950 border border-zinc-800 rounded-full relative overflow-hidden">
                    <div 
                      className="absolute top-0 h-full bg-cyan-400 transition-all duration-200"
                      style={{
                        width: `${Math.min(100, Math.abs(telemetry.lane_error) * 2)}%`,
                        left: telemetry.lane_error >= 0 ? "50%" : "auto",
                        right: telemetry.lane_error < 0 ? "50%" : "auto"
                      }}
                    />
                  </div>
                </div>
              </div>

            </CardContent>
          </Card>

          {/* Dynamic Tuning Sliders */}
          <Card className="bg-zinc-950 border-zinc-900 shadow-xl overflow-hidden">
            <CardHeader className="pb-2 pt-4">
              <CardTitle className="text-zinc-200 text-md font-semibold flex items-center gap-2">
                <Sliders className="h-4.5 w-4.5 text-cyan-400" /> Tuning & Parameters
              </CardTitle>
            </CardHeader>
            <CardContent className="flex flex-col gap-4 pb-5">
              
              {/* Speed Config */}
              <div className="space-y-1.5">
                <div className="flex justify-between items-center text-xs font-mono">
                  <span className="text-zinc-400">Motor Base Speed</span>
                  <span className="text-cyan-400 font-bold">{baseSpeed.toFixed(2)} m/s</span>
                </div>
                <Slider
                  min={0.1}
                  max={1.0}
                  step={0.05}
                  value={[baseSpeed]}
                  onValueChange={(val) => setBaseSpeed(val[0])}
                  onValueCommit={(val) => setRosParameter("brain_node", "base_speed", 3, val[0])}
                  className="py-1"
                />
              </div>

              {/* PID KP Config */}
              <div className="space-y-1.5">
                <div className="flex justify-between items-center text-xs font-mono">
                  <span className="text-zinc-400">Steering Gain Kp</span>
                  <span className="text-cyan-400 font-bold">{kp.toFixed(4)}</span>
                </div>
                <Slider
                  min={0.0}
                  max={0.02}
                  step={0.0005}
                  value={[kp]}
                  onValueChange={(val) => setKp(val[0])}
                  onValueCommit={(val) => setRosParameter("brain_node", "kp", 3, val[0])}
                  className="py-1"
                />
              </div>

              {/* PID KD Config */}
              <div className="space-y-1.5">
                <div className="flex justify-between items-center text-xs font-mono">
                  <span className="text-zinc-400">Steering Gain Kd</span>
                  <span className="text-cyan-400 font-bold">{kd.toFixed(4)}</span>
                </div>
                <Slider
                  min={0.0}
                  max={0.005}
                  step={0.0001}
                  value={[kd]}
                  onValueChange={(val) => setKd(val[0])}
                  onValueCommit={(val) => setRosParameter("brain_node", "kd", 3, val[0])}
                  className="py-1"
                />
              </div>

              {/* Nudge Duration Config */}
              <div className="space-y-1.5">
                <div className="flex justify-between items-center text-xs font-mono">
                  <span className="text-zinc-400">Nudge Duration</span>
                  <span className="text-cyan-400 font-bold">{nudgeDuration.toFixed(1)} s</span>
                </div>
                <Slider
                  min={0.5}
                  max={3.0}
                  step={0.1}
                  value={[nudgeDuration]}
                  onValueChange={(val) => setNudgeDuration(val[0])}
                  onValueCommit={(val) => setRosParameter("brain_node", "nudge_duration", 3, val[0])}
                  className="py-1"
                />
              </div>

              <div className="border-t border-zinc-900 my-1" />

              {/* Front danger zone */}
              <div className="space-y-1.5">
                <div className="flex justify-between items-center text-xs font-mono">
                  <span className="text-zinc-400">Lidar Front Safe Distance</span>
                  <span className="text-yellow-400 font-bold">{frontDangerZone.toFixed(2)} m</span>
                </div>
                <Slider
                  min={0.2}
                  max={1.5}
                  step={0.05}
                  value={[frontDangerZone]}
                  onValueChange={(val) => setFrontDangerZone(val[0])}
                  onValueCommit={(val) => setRosParameter("lidar_monitor_node", "front_danger_zone", 3, val[0])}
                  className="py-1"
                />
              </div>

              {/* Side safe zone */}
              <div className="space-y-1.5">
                <div className="flex justify-between items-center text-xs font-mono">
                  <span className="text-zinc-400">Lidar Side Safe Distance</span>
                  <span className="text-yellow-400 font-bold">{sideSafeZone.toFixed(2)} m</span>
                </div>
                <Slider
                  min={0.2}
                  max={1.5}
                  step={0.05}
                  value={[sideSafeZone]}
                  onValueChange={(val) => setSideSafeZone(val[0])}
                  onValueCommit={(val) => setRosParameter("lidar_monitor_node", "side_safe_zone", 3, val[0])}
                  className="py-1"
                />
              </div>

              {/* Lidar Offset Angle */}
              <div className="space-y-1.5">
                <div className="flex justify-between items-center text-xs font-mono">
                  <span className="text-zinc-400">Lidar Mounting Rotation</span>
                  <span className="text-indigo-400 font-bold">{lidarOffsetDeg.toFixed(1)}°</span>
                </div>
                <Slider
                  min={-180}
                  max={180}
                  step={1.0}
                  value={[lidarOffsetDeg]}
                  onValueChange={(val) => setLidarOffsetDeg(val[0])}
                  onValueCommit={(val) => setRosParameter("lidar_monitor_node", "lidar_offset_deg", 3, val[0])}
                  className="py-1"
                />
              </div>

            </CardContent>
          </Card>

        </div>
      </div>
    </div>
  );
}
