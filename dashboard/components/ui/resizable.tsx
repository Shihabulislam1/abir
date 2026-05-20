"use client"

import { Group, Panel, Separator } from "react-resizable-panels"
import { cn } from "@/lib/utils"

// ResizablePanelGroup → react-resizable-panels Group
// The library expects `orientation` ("horizontal" | "vertical").
// We accept both `direction` (shadcn convention) and `orientation` so
// either spelling works for callers.
function ResizablePanelGroup({
  className,
  direction,
  orientation,
  ...props
}: {
  className?: string
  direction?: "horizontal" | "vertical"
  orientation?: "horizontal" | "vertical"
  [key: string]: any
}) {
  const resolvedOrientation = orientation ?? direction ?? "horizontal"
  return (
    <Group
      data-slot="resizable-panel-group"
      className={cn(
        "flex h-full w-full data-[orientation=vertical]:flex-col",
        className
      )}
      orientation={resolvedOrientation}
      {...props}
    />
  )
}

// ResizablePanel → react-resizable-panels Panel
function ResizablePanel({ className, ...props }: { className?: string; [key: string]: any }) {
  return (
    <Panel
      data-slot="resizable-panel"
      className={cn("overflow-hidden", className)}
      {...props}
    />
  )
}

// ResizableHandle → react-resizable-panels Separator
function ResizableHandle({
  withHandle = false,
  className,
  ...props
}: {
  withHandle?: boolean
  className?: string
  [key: string]: any
}) {
  return (
    <Separator
      data-slot="resizable-handle"
      className={cn(
        // Vertical separator (between horizontal panels) → thin vertical bar
        "relative flex shrink-0 items-center justify-center bg-zinc-800",
        // The orientation aria attr is set by the library automatically
        "data-[orientation=horizontal]:h-[6px] data-[orientation=horizontal]:w-full data-[orientation=horizontal]:cursor-row-resize",
        "data-[orientation=vertical]:h-full data-[orientation=vertical]:w-[6px] data-[orientation=vertical]:cursor-col-resize",
        "hover:bg-cyan-500/30 active:bg-cyan-500/50 focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-cyan-400",
        "transition-colors duration-150",
        className
      )}
      {...props}
    >
      {withHandle && (
        <div
          className={cn(
            "z-10 flex rounded-full bg-zinc-600",
            // Knob shape adapts to orientation
            "data-[orientation=horizontal]:h-1 data-[orientation=horizontal]:w-8",
            "data-[orientation=vertical]:h-8 data-[orientation=vertical]:w-1"
          )}
        />
      )}
    </Separator>
  )
}

export { ResizableHandle, ResizablePanel, ResizablePanelGroup }
