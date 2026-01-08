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

export const ControlPanel = ({ sendPid, sendSetpoint, sendCalibration, distance = 0, externalSetpoint = 150, rawDistance = 0 }) => {
    const [controlMode, setControlMode] = useState("PID");
    const [setpoint, setSetpoint] = useState(145); // Updated to beam center (290/2)
    const [kp, setKp] = useState(0.15); // Updated to match firmware
    const [ki, setKi] = useState(0.0025); // Updated to match firmware
    const [kd, setKd] = useState(6.0); // Updated to match firmware

    // Calibration state
    const [calPoints, setCalPoints] = useState([null, null, null, null, null]); // 5 points
    const calTargets = [0, 75, 150, 225, 290]; // Target positions in mm
    const [calStatus, setCalStatus] = useState("Gotowy do kalibracji");

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

    const handleCalibratePoint = (index, targetPos) => {
        if (rawDistance === 0) {
            setCalStatus(`❌ Błąd: Brak odczytu RAW! Poczekaj na dane z czujnika.`);
            return;
        }

        const newCalPoints = [...calPoints];
        newCalPoints[index] = rawDistance;
        setCalPoints(newCalPoints);

        const completed = newCalPoints.filter((p) => p !== null).length;
        setCalStatus(`✅ Punkt ${index + 1}/5: ${targetPos}mm → RAW:${rawDistance.toFixed(0)} | Zebrano: ${completed}/5`);

        if (completed === 5) {
            setCalStatus(`🎉 Wszystkie 5 punktów zapisane! Kliknij 'Zapisz do STM32'`);
        }
    };

    const handleSaveCalibration = async () => {
        if (calPoints.includes(null)) {
            const missing = calPoints.map((p, i) => (p === null ? i : null)).filter((i) => i !== null);
            setCalStatus(`❌ Brak punktów: ${missing.join(", ")}`);
            return;
        }

        try {
            for (let i = 0; i < 5; i++) {
                await sendCalibration(i, calPoints[i], calTargets[i]);
                await new Promise((resolve) => setTimeout(resolve, 600)); // Safe delay for Web Serial -> STM32
            }
            setCalStatus("✅ Kalibracja wysłana do STM32!");
        } catch (error) {
            setCalStatus(`❌ Błąd: ${error.message}`);
        }
    };

    return (
        <div className="control-panel">
            <h3 className="panel-title">Panel Sterowania</h3>

            {/* Visualizer inside Control Panel */}
            <BeamVisualizer distance={distance} setpoint={setpoint} onSetpointChange={handleVizChange} />

            {/* Calibration Panel */}
            <div className="calibration-panel" style={{ marginBottom: "1rem" }}>
                <h4 style={{ fontSize: "0.75rem", fontWeight: 700, textTransform: "uppercase", marginBottom: "0.75rem", color: "var(--warning)" }}>Kalibracja 5-Punktowa</h4>
                {/* <p style={{ fontSize: "0.7rem", color: "var(--text-muted)", marginBottom: "0.5rem" }}>Połóż piłkę w pozycji i kliknij przycisk:</p> */}

                <div style={{ display: "flex", gap: "0.4rem", marginBottom: "0.75rem", flexWrap: "nowrap" }}>
                    {calTargets.map((target, idx) => {
                        const colors = ["#ff4444", "#ff8800", "#ffcc00", "#88ff00", "#00ff00"];
                        const labels = ["Start", "25%", "Środek", "75%", "Koniec"];

                        return (
                            <button
                                key={idx}
                                onClick={() => handleCalibratePoint(idx, target)}
                                className="btn"
                                style={{
                                    flex: "1",
                                    minWidth: "0",
                                    padding: "0.6rem 0.2rem",
                                    background: calPoints[idx] !== null ? `linear-gradient(135deg, ${colors[idx]}, ${colors[idx]}cc)` : "var(--bg-card)",
                                    border: calPoints[idx] !== null ? `2px solid ${colors[idx]}` : "1px solid var(--border-color)",
                                    borderRadius: "8px",
                                    color: calPoints[idx] !== null ? "black" : "var(--text-secondary)",
                                    fontWeight: "600",
                                    fontSize: "0.7rem",
                                    cursor: "pointer",
                                    transition: "all 0.2s",
                                    boxShadow: calPoints[idx] !== null ? `0 0 10px ${colors[idx]}40` : "none",
                                    whiteSpace: "nowrap",
                                    overflow: "hidden",
                                }}
                            >
                                <div style={{ fontWeight: "700" }}>
                                    {labels[idx]}
                                    {calPoints[idx] !== null && <span style={{ fontWeight: "400", opacity: 0.9 }}> - {calPoints[idx].toFixed(0)}</span>}
                                </div>
                            </button>
                        );
                    })}
                </div>

                {/* Status text area (Hidden if success, shown only for errors/info) */}
                {calStatus && !calStatus.includes("wysłana") && (
                    <div
                        style={{ fontSize: "0.7rem", color: "var(--text-secondary)", marginBottom: "0.5rem", minHeight: "2rem", padding: "0.4rem", background: "var(--bg-app)", borderRadius: "4px", border: "1px solid var(--border-color)" }}
                    >
                        {calStatus}
                    </div>
                )}

                <button
                    onClick={handleSaveCalibration}
                    disabled={calPoints.includes(null)}
                    className="btn btn-primary"
                    style={{
                        width: "100%",
                        padding: "0.6rem",
                        background: calPoints.includes(null) ? "var(--bg-card)" : calStatus.includes("wysłana") ? "#00cc00" : "var(--success)",
                        color: calPoints.includes(null) ? "var(--text-muted)" : calStatus.includes("wysłana") ? "white" : "white",
                        border: calPoints.includes(null) ? "1px solid var(--border-color)" : "none",
                        borderRadius: "8px",
                        fontWeight: "700",
                        fontSize: "0.8rem",
                        cursor: calPoints.includes(null) ? "not-allowed" : "pointer",
                        opacity: calPoints.includes(null) ? 0.5 : 1,
                        transition: "all 0.2s",
                        boxShadow: calPoints.includes(null) ? "none" : "0 0 15px var(--success)40",
                        transform: calStatus.includes("wysłana") ? "scale(1.02)" : "scale(1)",
                    }}
                >
                    {calStatus.includes("wysłana") ? "Wysłano kalibrację!" : "Zapisz kalibrację"}
                </button>
            </div>

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
