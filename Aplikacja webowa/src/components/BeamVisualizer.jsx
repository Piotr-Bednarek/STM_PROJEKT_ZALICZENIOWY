import React, { useEffect, useRef, useState } from "react";

export const BeamVisualizer = ({ distance, setpoint, onSetpointChange, min = 0, max = 300 }) => {
    const svgRef = useRef(null);
    const [isDragging, setIsDragging] = useState(false);

    // Helpers to convert mm to % (0-100)
    const toPercent = (val) => ((Math.max(min, Math.min(val, max)) - min) / (max - min)) * 100;

    const handleInteraction = (clientX) => {
        if (!svgRef.current) return;
        const rect = svgRef.current.getBoundingClientRect();
        const x = clientX - rect.left;
        const width = rect.width;

        let newVal = (x / width) * (max - min) + min;
        newVal = Math.max(min, Math.min(newVal, max));

        onSetpointChange(newVal);
    };

    const handleMouseDown = (e) => {
        setIsDragging(true);
        handleInteraction(e.clientX);
    };

    const handleMouseMove = (e) => {
        if (isDragging) {
            handleInteraction(e.clientX);
        }
    };

    const handleMouseUp = () => {
        setIsDragging(false);
    };

    useEffect(() => {
        window.addEventListener("mouseup", handleMouseUp);
        return () => window.removeEventListener("mouseup", handleMouseUp);
    }, []);

    // Layout Constants
    const beamY = 70; // % from top
    const beamHeight = 6; // px thickness
    const ballRadius = 15; // px

    return (
        <div className="beam-viz-container" style={{ margin: "0.5rem 0 1.5rem 0" }}>
            <div
                style={{
                    position: "relative",
                    height: "120px",
                    width: "100%",
                    cursor: "pointer",
                    userSelect: "none",
                }}
                onMouseDown={handleMouseDown}
                onMouseMove={handleMouseMove}
            >
                {/* SVG Container */}
                <svg ref={svgRef} width="100%" height="100%" style={{ overflow: "visible" }}>
                    <defs>
                        <linearGradient id="beamGradientSide" x1="0%" y1="0%" x2="0%" y2="100%">
                            <stop offset="0%" stopColor="#666" />
                            <stop offset="100%" stopColor="#444" />
                        </linearGradient>

                        <radialGradient id="ballGradientSide" cx="30%" cy="30%" r="70%">
                            <stop offset="0%" stopColor="#88ff88" />
                            <stop offset="100%" stopColor="#00aa00" />
                        </radialGradient>

                        <filter id="shadowSide">
                            <feDropShadow dx="0" dy="4" stdDeviation="4" floodColor="#000" floodOpacity="0.5" />
                        </filter>
                    </defs>

                    {/* --- BEAM (Side View) --- */}
                    {/* Main Rail */}
                    <rect x="0%" y={`${beamY}%`} width="100%" height={beamHeight} fill="url(#beamGradientSide)" rx="2" />

                    {/* Fulcrum (Pivot point at center) - optional decoration */}
                    <path d="M 50%,70% L 48%,90% L 52%,90% Z" fill="#555" />

                    {/* --- BALL --- */}
                    {/* Needs to sit ON TOP of the beam. 
                        Cy calculation: beamY% (approx) - radius
                        Let's use calc or simple alignment because SVG y is usually px or %.
                        If height is 120px, 70% is 84px. Ball radius 15px. Center should be 84-15-half_beam = ~65px?
                        Let's approximate: y="55%"
                    */}
                    <circle
                        cx={`${toPercent(distance)}%`}
                        cy={`${beamY - 12}%`}
                        r={ballRadius}
                        fill="url(#ballGradientSide)"
                        stroke="#005500"
                        strokeWidth="1"
                        filter="url(#shadowSide)"
                        style={{ transition: "cx 0.05s linear" }}
                    />

                    {/* --- SETPOINT MARKER --- */}
                    {/* A flag or arrow ABOVE the ball path */}
                    {/* --- SETPOINT MARKER (Ghost Ball) --- */}
                    <g style={{ transition: "all 0.1s ease-out" }}>
                        {/* Vertical Guide Line */}
                        <line
                            x1={`${toPercent(setpoint)}%`}
                            y1="10%"
                            x2={`${toPercent(setpoint)}%`}
                            y2={`${beamY + 12}%`}
                            stroke="#f39c12"
                            strokeWidth="1"
                            strokeDasharray="2 2"
                            opacity="0.4"
                        />

                        {/* Ghost Ball (Hollow/Transparent) */}
                        <circle
                            cx={`${toPercent(setpoint)}%`}
                            cy={`${beamY - 12}%`}
                            r={ballRadius}
                            fill="rgba(243, 156, 18, 0.2)"
                            stroke="#f39c12"
                            strokeWidth="2"
                            strokeDasharray="4 2"
                        />

                        {/* Center Dot for precision */}
                        <circle cx={`${toPercent(setpoint)}%`} cy={`${beamY - 12}%`} r="2" fill="#f39c12" />
                    </g>

                    {/* Ruler Marks */}
                    {[0, 50, 100, 150, 200, 250, 300].map((mark) => (
                        <g key={mark}>
                            <rect x={`${toPercent(mark)}%`} y={`${beamY + 5}%`} width="1" height="5" fill="#666" transform="translate(-0.5,0)" />
                            {mark % 100 === 0 && (
                                <text x={`${toPercent(mark)}%`} y={`${beamY + 18}%`} fill="#666" fontSize="10" fontFamily="monospace" textAnchor="middle">
                                    {mark}
                                </text>
                            )}
                        </g>
                    ))}
                </svg>
            </div>

            <div
                style={{
                    display: "flex",
                    justifyContent: "space-between",
                    marginTop: "-10px",
                    padding: "0.5rem 1rem",
                    background: "rgba(255,255,255,0.03)",
                    borderRadius: "8px",
                    fontSize: "0.80rem",
                }}
            >
                <span style={{ color: "var(--text-muted)" }}>0 mm</span>
                <span>
                    Aktualnie: <strong style={{ color: "#2ecc71" }}>{distance.toFixed(0)}</strong>
                    <span style={{ margin: "0 8px", color: "#555" }}>|</span>
                    Cel: <strong style={{ color: "#f39c12" }}>{setpoint.toFixed(0)}</strong>
                </span>
                <span style={{ color: "var(--text-muted)" }}>300 mm</span>
            </div>
        </div>
    );
};
