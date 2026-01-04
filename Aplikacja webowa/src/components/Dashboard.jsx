import { Activity, AlertCircle, PlugZap, Usb } from "lucide-react";
import React from "react";
import "../App.css"; // Ensure styles are loaded
import { useSerial } from "../hooks/useSerial";
import { BeamVisualizer } from "./BeamVisualizer";
import { Charts } from "./Charts";
import { ControlPanel } from "./ControlPanel";
import { Metrics } from "./Metrics";
import { Terminal } from "./Terminal";

export const Dashboard = () => {
    const { isConnected, connect, disconnect, logs, latestData, dataHistory, sendSetpoint, sendPid } = useSerial();

    return (
        <div className="app-container">
            {/* Header */}
            <header className="app-header">
                <div className="header-left">
                    <div className="logo-icon">
                        <Activity size={24} />
                    </div>
                    <div className="header-title">
                        <h1>Panel Sterowania STM32</h1>
                        <div className="header-subtitle">
                            <span className="indicator-dot"></span>
                            <span>Interfejs Serial</span>
                        </div>
                    </div>
                </div>

                <div className="header-right">
                    <div className={`status-badge ${isConnected ? "connected" : "disconnected"}`}>
                        <div className="pulse-dot"></div>
                        <span>{isConnected ? "POŁĄCZONO" : "ROZŁĄCZONO"}</span>
                    </div>

                    <button onClick={isConnected ? disconnect : connect} className={`btn ${isConnected ? "btn-danger" : "btn-primary"}`}>
                        {isConnected ? <PlugZap size={18} /> : <Usb size={18} />}
                        {isConnected ? "Rozłącz" : "Połącz"}
                    </button>
                </div>
            </header>

            {/* Main Content */}
            <main className="main-content">
                <div className="dashboard-grid">
                    {/* Left Column: Controls & Metrics */}
                    <div className="left-panel custom-scrollbar">
                        {/* Status Cards */}
                        <Metrics data={latestData} />

                        {/* Control Panel */}
                        <ControlPanel sendSetpoint={sendSetpoint} sendPid={sendPid} distance={latestData.filtered} externalSetpoint={latestData.setpoint} />

                        {/* Terminal */}
                        <div style={{ flex: 1, minHeight: "200px" }}>
                            <Terminal logs={logs} />
                        </div>
                    </div>

                    {/* Right Column: Charts */}
                    <div className="right-panel">
                        {!isConnected && dataHistory.length === 0 ? (
                            <div
                                style={{
                                    height: "100%",
                                    display: "flex",
                                    flexDirection: "column",
                                    alignItems: "center",
                                    justifyContent: "center",
                                    color: "var(--text-muted)",
                                    gap: "1rem",
                                }}
                            >
                                <AlertCircle size={48} style={{ opacity: 0.2 }} />
                                <p>Połącz z urządzeniem STM32 aby rozpocząć transmisję.</p>
                            </div>
                        ) : (
                            <Charts dataHistory={dataHistory} />
                        )}
                    </div>
                </div>
            </main>
        </div>
    );
};
