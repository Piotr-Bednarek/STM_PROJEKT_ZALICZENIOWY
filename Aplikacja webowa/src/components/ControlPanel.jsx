import React, { useState } from "react";

const SliderControl = ({ label, value, onChange, min, max, step, onCommit, unit }) => {
    return (
        <div className="slider-container">
            <div className="slider-header">
                <label className="slider-label">{label}</label>
                <span className="slider-value">
                    {value} {unit}
                </span>
            </div>
            <input type="range" min={min} max={max} step={step} value={value} onChange={(e) => onChange(parseFloat(e.target.value))} onMouseUp={() => onCommit && onCommit(value)} onTouchEnd={() => onCommit && onCommit(value)} />
        </div>
    );
};

import { BeamVisualizer } from "./BeamVisualizer";

export const ControlPanel = ({ sendPid, sendSetpoint, distance = 0, externalSetpoint = 150 }) => {
    const [controlMode, setControlMode] = useState("PID");
    const [setpoint, setSetpoint] = useState(145); // Updated to beam center (290/2)
    const [kp, setKp] = useState(0.29); // Updated to match firmware
    const [ki, setKi] = useState(0.0003); // Updated to match firmware
    const [kd, setKd] = useState(9.0); // Updated to match firmware

    // Sync local setpoint state with external (optimistic) updates or if connection resets
    React.useEffect(() => {
        if (externalSetpoint !== undefined) {
            setSetpoint(externalSetpoint);
        }
    }, [externalSetpoint]);

    const handlePidCommit = () => {
        sendPid(kp, ki, kd); // Send positive values to STM32 (main.c no longer inverts)
    };

    const handleVizChange = (val) => {
        setSetpoint(val);
        sendSetpoint(val);
    };

    return (
        <div className="control-panel">
            <h3 className="panel-title">Panel Sterowania</h3>

            {/* Visualizer inside Control Panel */}
            <BeamVisualizer distance={distance} setpoint={setpoint} onSetpointChange={handleVizChange} />

            <div className="space-y-4">
                <div className="control-group">
                    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "1rem" }}>
                        <h4 className="group-title" style={{ color: "var(--primary)", margin: 0 }}>
                            Parametry {controlMode}
                        </h4>

                        {/* PID/LQR Switch */}
                        <div
                            style={{
                                display: "flex",
                                alignItems: "center",
                                background: "var(--bg-app)",
                                borderRadius: "6px",
                                padding: "4px",
                                border: "1px solid var(--border-color)",
                            }}
                        >
                            <button
                                onClick={() => setControlMode("PID")}
                                style={{
                                    padding: "4px 12px",
                                    borderRadius: "4px",
                                    cursor: "pointer",
                                    fontSize: "0.75rem",
                                    fontWeight: "700",
                                    border: "none",
                                    background: controlMode === "PID" ? "var(--primary)" : "transparent",
                                    color: controlMode === "PID" ? "white" : "var(--text-secondary)",
                                    transition: "all 0.2s",
                                }}
                            >
                                PID
                            </button>
                            <button
                                onClick={() => setControlMode("LQR")}
                                style={{
                                    padding: "4px 12px",
                                    borderRadius: "4px",
                                    cursor: "pointer",
                                    fontSize: "0.75rem",
                                    fontWeight: "700",
                                    border: "none",
                                    background: controlMode === "LQR" ? "var(--primary)" : "transparent",
                                    color: controlMode === "LQR" ? "white" : "var(--text-secondary)",
                                    transition: "all 0.2s",
                                }}
                            >
                                LQR
                            </button>
                        </div>
                    </div>
                    <SliderControl label="Kp (Proporcjonalny)" value={kp} min={0.0} max={2.0} step={0.01} onChange={setKp} onCommit={(val) => sendPid(-val, -ki, -kd)} />
                    <SliderControl label="Ki (Całkujący)" value={ki} min={0.0} max={0.01} step={0.0001} onChange={setKi} onCommit={(val) => sendPid(-kp, -val, -kd)} />
                    <SliderControl label="Kd (Różniczkujący)" value={kd} min={0.0} max={10.0} step={0.1} onChange={setKd} onCommit={(val) => sendPid(-kp, -ki, -val)} />

                    <button onClick={handlePidCommit} className="btn btn-primary" style={{ width: "100%", justifyContent: "center", marginTop: "1rem" }}>
                        Wymuś Wyślij PID
                    </button>
                </div>
            </div>
        </div>
    );
};
