"use client";

import * as React from "react";
import * as ROSLIB from "roslib";
import { Radar, ZoomIn, ZoomOut, RotateCcw } from "lucide-react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";

const WS_URL = `ws://${process.env.NEXT_PUBLIC_ROSBRIDGE_URI ?? "localhost:9090"}`;

type ConnStatus = "disconnected" | "connecting" | "connected" | "error";

const STATUS_STYLES: Record<ConnStatus, string> = {
  disconnected: "bg-zinc-700 text-zinc-300",
  connecting: "bg-yellow-500/20 text-yellow-400 animate-pulse",
  connected: "bg-green-500/20 text-green-400",
  error: "bg-red-500/20 text-red-400",
};

const STATUS_LABELS: Record<ConnStatus, string> = {
  disconnected: "OFFLINE",
  connecting: "CONNECTING",
  connected: "LIVE",
  error: "ERROR",
};

interface LaserScan {
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  ranges: number[];
}

interface LidarVisualizationProps {
  ros?: ROSLIB.Ros | null;
}

export function LidarVisualization({ ros: externalRos }: LidarVisualizationProps) {
  const canvasRef = React.useRef<HTMLCanvasElement>(null);
  const containerRef = React.useRef<HTMLDivElement>(null);
  const rosRef = React.useRef<ROSLIB.Ros | null>(null);
  const subRef = React.useRef<ROSLIB.Topic<LaserScan> | null>(null);
  const lastScanRef = React.useRef<LaserScan | null>(null);
  const animFrameRef = React.useRef<number | null>(null);

  const [status, setStatus] = React.useState<ConnStatus>("disconnected");
  const [scanHz, setScanHz] = React.useState<number>(0);
  const [pointCount, setPointCount] = React.useState<number>(0);
  const [zoom, setZoom] = React.useState<number>(50); // pixels per meter
  const topic = "/scan";

  // Hz counter
  const hzCountRef = React.useRef<number>(0);
  React.useEffect(() => {
    const id = setInterval(() => {
      setScanHz(hzCountRef.current);
      hzCountRef.current = 0;
    }, 1000);
    return () => clearInterval(id);
  }, []);

  // Canvas draw loop
  React.useEffect(() => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    const draw = () => {
      // Resize canvas to container
      const { clientWidth: w, clientHeight: h } = container;
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }

      const ctx = canvas.getContext("2d");
      if (!ctx) return;

      const cx = w / 2;
      const cy = h / 2;

      // Background
      ctx.fillStyle = "#09090b";
      ctx.fillRect(0, 0, w, h);

      // Grid rings
      const maxRingMeters = Math.ceil(Math.min(w, h) / 2 / zoom);
      for (let r = 1; r <= maxRingMeters; r++) {
        const px = r * zoom;
        ctx.beginPath();
        ctx.arc(cx, cy, px, 0, Math.PI * 2);
        ctx.strokeStyle = r % 5 === 0 ? "#3f3f46" : "#27272a";
        ctx.lineWidth = r % 5 === 0 ? 1 : 0.5;
        ctx.stroke();
        // Distance label
        if (r % 2 === 0) {
          ctx.fillStyle = "#52525b";
          ctx.font = "10px monospace";
          ctx.fillText(`${r}m`, cx + px + 3, cy - 3);
        }
      }

      // Cross-hairs
      ctx.strokeStyle = "#27272a";
      ctx.lineWidth = 0.5;
      ctx.beginPath();
      ctx.moveTo(cx, 0);
      ctx.lineTo(cx, h);
      ctx.moveTo(0, cy);
      ctx.lineTo(w, cy);
      ctx.stroke();

      // Cardinal labels
      ctx.fillStyle = "#52525b";
      ctx.font = "11px monospace";
      ctx.fillText("F", cx + 4, 14);
      ctx.fillText("B", cx + 4, h - 4);
      ctx.fillText("L", 4, cy - 4);
      ctx.fillText("R", w - 14, cy - 4);

      // Robot dot
      ctx.beginPath();
      ctx.arc(cx, cy, 6, 0, Math.PI * 2);
      ctx.fillStyle = "#22d3ee";
      ctx.fill();
      // Direction arrow
      ctx.beginPath();
      ctx.moveTo(cx, cy - 6);
      ctx.lineTo(cx - 5, cy + 4);
      ctx.lineTo(cx + 5, cy + 4);
      ctx.closePath();
      ctx.fillStyle = "#22d3ee";
      ctx.fill();

      // Scan points
      const scan = lastScanRef.current;
      if (!scan) return;

      const { angle_min, angle_increment, range_min, range_max, ranges } = scan;

      ctx.fillStyle = "#4ade80";
      for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i];
        if (!isFinite(r) || r < range_min || r > range_max) continue;

        // ROS convention: angle 0 = forward (+Y in canvas = down, so forward = -Y)
        const angle = angle_min + i * angle_increment;
        const px = cx + r * zoom * Math.sin(angle);
        const py = cy - r * zoom * Math.cos(angle);

        ctx.beginPath();
        ctx.arc(px, py, 1.5, 0, Math.PI * 2);
        ctx.fill();
      }

      animFrameRef.current = requestAnimationFrame(draw);
    };

    animFrameRef.current = requestAnimationFrame(draw);
    return () => {
      if (animFrameRef.current) cancelAnimationFrame(animFrameRef.current);
    };
  }, [zoom]);

  // ROS connection & subscription
  React.useEffect(() => {
    if (externalRos) {
      setStatus(externalRos.isConnected ? "connected" : "connecting");

      const sub = new ROSLIB.Topic<LaserScan>({
        ros: externalRos,
        name: topic,
        messageType: "sensor_msgs/LaserScan",
      });
      subRef.current = sub;

      sub.subscribe((scan) => {
        lastScanRef.current = scan;
        hzCountRef.current++;
        // Count valid points
        const valid = scan.ranges.filter(
          (r) => isFinite(r) && r >= scan.range_min && r <= scan.range_max,
        ).length;
        setPointCount(valid);
      });

      const handleConnect = () => setStatus("connected");
      const handleError = () => setStatus("error");
      const handleClose = () => setStatus("disconnected");

      externalRos.on("connection", handleConnect);
      externalRos.on("error", handleError);
      externalRos.on("close", handleClose);

      return () => {
        sub.unsubscribe();
        externalRos.off("connection", handleConnect);
        externalRos.off("error", handleError);
        externalRos.off("close", handleClose);
        lastScanRef.current = null;
      };
    }

    setStatus("connecting");

    const ros = new ROSLIB.Ros({ url: WS_URL });
    rosRef.current = ros;

    ros.on("connection", () => setStatus("connected"));
    ros.on("error", () => setStatus("error"));
    ros.on("close", () => setStatus("disconnected"));

    const sub = new ROSLIB.Topic<LaserScan>({
      ros,
      name: topic,
      messageType: "sensor_msgs/LaserScan",
    });
    subRef.current = sub;

    sub.subscribe((scan) => {
      lastScanRef.current = scan;
      hzCountRef.current++;
      // Count valid points
      const valid = scan.ranges.filter(
        (r) => isFinite(r) && r >= scan.range_min && r <= scan.range_max,
      ).length;
      setPointCount(valid);
    });

    return () => {
      sub.unsubscribe();
      ros.close();
      lastScanRef.current = null;
    };
  }, [externalRos]);

  return (
    <div className="flex flex-col h-full bg-background">
      {/* Header */}
      <div className="flex items-center justify-between px-3 py-2 border-b shrink-0">
        <div className="flex items-center gap-2">
          <Radar className="h-4 w-4 text-cyan-400" />
          <span className="text-sm font-semibold">LiDAR Scan</span>
          <span className="ml-2 text-xs text-muted-foreground font-mono">
            {topic}
          </span>
        </div>
        <div className="flex items-center gap-2">
          <span className="text-xs text-muted-foreground font-mono">
            {pointCount} pts
          </span>
          <span className="text-xs text-muted-foreground font-mono">
            {scanHz} Hz
          </span>
          <Badge
            className={`text-xs px-2 py-0.5 ${STATUS_STYLES[status]}`}
            variant="outline"
          >
            {STATUS_LABELS[status]}
          </Badge>
        </div>
      </div>

      {/* Canvas */}
      <div ref={containerRef} className="flex-1 relative overflow-hidden">
        <canvas ref={canvasRef} className="absolute inset-0 w-full h-full" />

        {/* No data overlay */}
        {status !== "connected" && (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-muted-foreground bg-zinc-950/80">
            <Radar className="h-12 w-12 mb-3 opacity-20" />
            <p className="text-sm font-medium">
              {status === "connecting"
                ? "Connecting to rosbridge..."
                : status === "error"
                  ? "Connection error"
                  : "Disconnected"}
            </p>
            <p className="text-xs mt-1 opacity-60">{WS_URL}</p>
          </div>
        )}
      </div>

      {/* Zoom controls */}
      <div className="flex items-center gap-2 px-3 py-2 border-t shrink-0">
        <span className="text-xs text-muted-foreground">Zoom</span>
        <Button
          variant="outline"
          size="icon"
          className="h-6 w-6"
          onClick={() => setZoom((z) => Math.max(10, z - 10))}
        >
          <ZoomOut className="h-3 w-3" />
        </Button>
        <span className="text-xs font-mono w-16 text-center">{zoom} px/m</span>
        <Button
          variant="outline"
          size="icon"
          className="h-6 w-6"
          onClick={() => setZoom((z) => Math.min(200, z + 10))}
        >
          <ZoomIn className="h-3 w-3" />
        </Button>
        <Button
          variant="outline"
          size="icon"
          className="h-6 w-6 ml-1"
          onClick={() => setZoom(50)}
          title="Reset zoom"
        >
          <RotateCcw className="h-3 w-3" />
        </Button>
      </div>
    </div>
  );
}