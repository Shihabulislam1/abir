"use client";

import * as React from "react";
import * as ROSLIB from "roslib";
import { Radar, ZoomIn, ZoomOut, RotateCcw, RotateCw, Compass } from "lucide-react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";

const WS_URL = `ws://${process.env.NEXT_PUBLIC_ROSBRIDGE_URI ?? "localhost:9090"}`;

// LiDAR viewing angle filter configuration (in degrees)
const MIN_VIEW_ANGLE_DEG = 165;
const MAX_VIEW_ANGLE_DEG = -165;

const MIN_VIEW_ANGLE_RAD = (MIN_VIEW_ANGLE_DEG * Math.PI) / 180;
const MAX_VIEW_ANGLE_RAD = (MAX_VIEW_ANGLE_DEG * Math.PI) / 180;

// Helper to check if an angle is within a range, handling wrap-around at 180/-180 degrees
const isAngleInRange = (angleRad: number, minRad: number, maxRad: number) => {
  let norm = angleRad;
  while (norm > Math.PI) norm -= 2 * Math.PI;
  while (norm < -Math.PI) norm += 2 * Math.PI;

  if (minRad <= maxRad) {
    return norm >= minRad && norm <= maxRad;
  } else {
    return norm >= minRad || norm <= maxRad;
  }
};

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
    const [zoom, setZoom] = React.useState<number>(120); // default zoom (pixels per meter, 1.2px per cm)
    const [rotation, setRotation] = React.useState<number>(0); // visualization rotation in degrees
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
            const diag = Math.sqrt(w * w + h * h) / 2;

            // Background (drawn static relative to canvas)
            ctx.fillStyle = "#09090b";
            ctx.fillRect(0, 0, w, h);

            // Save context, translate to center and apply rotation
            ctx.save();
            ctx.translate(cx, cy);
            ctx.rotate((rotation * Math.PI) / 180);

            // Grid rings (spaced at every 10cm / 0.1m)
            const stepMeters = 0.1;
            const maxRingMeters = Math.ceil(diag / zoom);
            for (let r = stepMeters; r <= maxRingMeters; r += stepMeters) {
                const px = r * zoom;
                ctx.beginPath();
                ctx.arc(0, 0, px, 0, Math.PI * 2);
                
                // Highlight every 50cm (0.5m) with a brighter color
                const isHalfMeter = Math.abs((r * 10) % 5) < 0.01;
                ctx.strokeStyle = isHalfMeter ? "#3f3f46" : "#1a1a1c";
                ctx.lineWidth = isHalfMeter ? 1 : 0.5;
                ctx.stroke();

                // Draw distance label on the 50cm intervals
                if (isHalfMeter) {
                    ctx.fillStyle = "#71717a";
                    ctx.font = "9px monospace";
                    ctx.fillText(`${Math.round(r * 100)}cm`, px + 3, -3);
                }
            }

            // Cross-hairs (drawn to cover rotated viewport)
            ctx.strokeStyle = "#27272a";
            ctx.lineWidth = 0.5;
            ctx.beginPath();
            ctx.moveTo(0, -diag);
            ctx.lineTo(0, diag);
            ctx.moveTo(-diag, 0);
            ctx.lineTo(diag, 0);
            ctx.stroke();

            // Cardinal labels
            ctx.fillStyle = "#52525b";
            ctx.font = "11px monospace";
            ctx.fillText("F", 4, -cy + 14);
            ctx.fillText("B", 4, cy - 4);
            ctx.fillText("L", -cx + 4, -4);
            ctx.fillText("R", cx - 14, -4);

            // Robot dot
            ctx.beginPath();
            ctx.arc(0, 0, 6, 0, Math.PI * 2);
            ctx.fillStyle = "#22d3ee";
            ctx.fill();
            
            // Direction arrow
            ctx.beginPath();
            ctx.moveTo(0, -6);
            ctx.lineTo(-5, 4);
            ctx.lineTo(5, 4);
            ctx.closePath();
            ctx.fillStyle = "#22d3ee";
            ctx.fill();

            // Draw cone boundary lines and shading based on configured angles
            const startAngleInCanvas = -Math.PI / 2 + MIN_VIEW_ANGLE_RAD;
            const endAngleInCanvas = -Math.PI / 2 + MAX_VIEW_ANGLE_RAD;
            
            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.arc(0, 0, diag * 2, startAngleInCanvas, endAngleInCanvas);
            ctx.closePath();
            ctx.fillStyle = "rgba(34, 211, 238, 0.05)"; // cyan tint for active sector
            ctx.fill();
            
            ctx.strokeStyle = "rgba(34, 211, 238, 0.15)";
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(diag * 2 * Math.sin(MIN_VIEW_ANGLE_RAD), -diag * 2 * Math.cos(MIN_VIEW_ANGLE_RAD));
            ctx.moveTo(0, 0);
            ctx.lineTo(diag * 2 * Math.sin(MAX_VIEW_ANGLE_RAD), -diag * 2 * Math.cos(MAX_VIEW_ANGLE_RAD));
            ctx.stroke();

            // Scan points (filtered to configured viewing angles)
            const scan = lastScanRef.current;
            if (scan) {
                const { angle_min, angle_increment, range_min, range_max, ranges } = scan;

                ctx.fillStyle = "#4ade80";
                for (let i = 0; i < ranges.length; i++) {
                    const r = ranges[i];
                    if (!isFinite(r) || r < range_min || r > range_max) continue;

                    // ROS convention: angle 0 = forward (+Y in canvas = down, so forward = -Y)
                    const angle = angle_min + i * angle_increment;

                    // Limit to configured viewing angle range
                    if (!isAngleInRange(angle, MIN_VIEW_ANGLE_RAD, MAX_VIEW_ANGLE_RAD)) continue;

                    const px = r * zoom * Math.sin(angle);
                    const py = -r * zoom * Math.cos(angle);

                    ctx.beginPath();
                    ctx.arc(px, py, 1.5, 0, Math.PI * 2);
                    ctx.fill();
                }
            }

            // Restore translated/rotated context state
            ctx.restore();

            animFrameRef.current = requestAnimationFrame(draw);
        };

        animFrameRef.current = requestAnimationFrame(draw);
        return () => {
            if (animFrameRef.current) cancelAnimationFrame(animFrameRef.current);
        };
    }, [zoom, rotation]);

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

                // Count valid points in the configured viewing angle range
                let valid = 0;
                for (let i = 0; i < scan.ranges.length; i++) {
                    const r = scan.ranges[i];
                    if (!isFinite(r) || r < scan.range_min || r > scan.range_max) continue;

                    const angle = scan.angle_min + i * scan.angle_increment;

                    if (isAngleInRange(angle, MIN_VIEW_ANGLE_RAD, MAX_VIEW_ANGLE_RAD)) {
                        valid++;
                    }
                }
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

            // Count valid points in the configured viewing angle range
            let valid = 0;
            for (let i = 0; i < scan.ranges.length; i++) {
                const r = scan.ranges[i];
                if (!isFinite(r) || r < scan.range_min || r > scan.range_max) continue;

                const angle = scan.angle_min + i * scan.angle_increment;

                if (isAngleInRange(angle, MIN_VIEW_ANGLE_RAD, MAX_VIEW_ANGLE_RAD)) {
                    valid++;
                }
            }
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

            {/* Zoom & Rotation controls */}
            <div className="flex items-center justify-between px-3 py-2 border-t shrink-0">
                <div className="flex items-center gap-2">
                    <span className="text-xs text-muted-foreground">Zoom</span>
                    <Button
                        variant="outline"
                        size="icon"
                        className="h-6 w-6"
                        onClick={() => setZoom((z) => Math.max(50, z - 50))}
                    >
                        <ZoomOut className="h-3 w-3" />
                    </Button>
                    <span className="text-xs font-mono w-16 text-center">{(zoom / 100).toFixed(1)} px/cm</span>
                    <Button
                        variant="outline"
                        size="icon"
                        className="h-6 w-6"
                        onClick={() => setZoom((z) => Math.min(1200, z + 50))}
                    >
                        <ZoomIn className="h-3 w-3" />
                    </Button>
                    <Button
                        variant="outline"
                        size="icon"
                        className="h-6 w-6 ml-1"
                        onClick={() => setZoom(120)}
                        title="Reset zoom"
                    >
                        <RotateCcw className="h-3 w-3" />
                    </Button>
                </div>

                <div className="flex items-center gap-2">
                    <span className="text-xs text-muted-foreground">Rotate</span>
                    <Button
                        variant="outline"
                        size="icon"
                        className="h-6 w-6"
                        onClick={() => setRotation((r) => (r - 15 + 360) % 360)}
                        title="Rotate Counter-Clockwise 15°"
                    >
                        <RotateCcw className="h-3 w-3" />
                    </Button>
                    <span className="text-xs font-mono w-12 text-center">{rotation}°</span>
                    <Button
                        variant="outline"
                        size="icon"
                        className="h-6 w-6"
                        onClick={() => setRotation((r) => (r + 15) % 360)}
                        title="Rotate Clockwise 15°"
                    >
                        <RotateCw className="h-3 w-3" />
                    </Button>
                    <Button
                        variant="outline"
                        size="icon"
                        className="h-6 w-6 ml-1"
                        onClick={() => setRotation(0)}
                        title="Reset Rotation"
                    >
                        <Compass className="h-3 w-3" />
                    </Button>
                </div>
            </div>
        </div>
    );
}