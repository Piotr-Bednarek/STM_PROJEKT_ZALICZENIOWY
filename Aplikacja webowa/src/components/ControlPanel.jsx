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
            <input
                type="range"
                min={min}
                max={max}
                step={step}
                value={value}
                onChange={(e) => onChange(parseFloat(e.target.value))}
                onMouseUp={() => onCommit && onCommit(value)}
                onTouchEnd={() => onCommit && onCommit(value)}
            />
        </div>
    );
};

export const ControlPanel = ({ sendPid, sendSetpoint }) => {
    const [setpoint, setSetpoint] = useState(150);
    const [kp, setKp] = useState(-0.11);
    const [ki, setKi] = useState(-0.004);
    const [kd, setKd] = useState(-2.0);

    const handlePidCommit = () => {
        sendPid(kp, ki, kd);
    };

    return (
        <div className="control-panel">
            <h3 className="panel-title">Panel Sterowania</h3>

            <div className="space-y-4">
                <div className="control-group">
                    <h4 className="group-title" style={{ color: "var(--chart-setpoint)" }}>
                        Cel (Setpoint)
                    </h4>
                    <SliderControl label="Poziom zadany" value={setpoint} min={0} max={300} step={1} unit="mm" onChange={setSetpoint} onCommit={(val) => sendSetpoint(val)} />
                </div>

                <div className="control-group">
                    <h4 className="group-title" style={{ color: "var(--primary)" }}>
                        Parametry PID
                    </h4>
                    <SliderControl label="Kp (Proporcjonalny)" value={kp} min={-2.0} max={0.0} step={0.01} onChange={setKp} onCommit={(val) => sendPid(val, ki, kd)} />
                    <SliderControl label="Ki (Całkujący)" value={ki} min={-0.01} max={0.0} step={0.0001} onChange={setKi} onCommit={(val) => sendPid(kp, val, kd)} />
                    <SliderControl label="Kd (Różniczkujący)" value={kd} min={-10.0} max={0.0} step={0.1} onChange={setKd} onCommit={(val) => sendPid(kp, ki, val)} />

                    <button onClick={handlePidCommit} className="btn btn-primary" style={{ width: "100%", justifyContent: "center", marginTop: "1rem" }}>
                        Wymuś Wyślij PID
                    </button>
                </div>
            </div>
        </div>
    );
};
