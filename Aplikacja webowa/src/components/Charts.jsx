import React from "react";
import { CartesianGrid, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";

// Custom tooltip for better performance and style
const CustomTooltip = ({ active, payload, label }) => {
    if (active && payload && payload.length) {
        return (
            <div style={{ backgroundColor: "var(--bg-card)", border: "1px solid var(--border-color)", padding: "0.5rem", borderRadius: "4px", boxShadow: "var(--shadow-lg)" }}>
                <div style={{ fontFamily: "var(--font-mono)", fontSize: "0.75rem" }}>
                    {payload.map((p) => (
                        <div key={p.name} style={{ color: p.color }}>
                            {p.name}: {p.value.toFixed(1)}
                        </div>
                    ))}
                    <div style={{ color: "var(--text-muted)", marginTop: "0.25rem" }}>Próbki temu: {label}</div>
                </div>
            </div>
        );
    }
    return null;
};

const CommonChart = ({ data, lines, height = 200, domain = ["auto", "auto"] }) => {
    if (!data || data.length === 0) {
        return (
            <div
                style={{
                    height,
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                    backgroundColor: "var(--bg-panel)",
                    borderRadius: "var(--radius-md)",
                    border: "1px solid var(--border-color)",
                    color: "var(--text-muted)",
                }}
            >
                Oczekiwanie na dane...
            </div>
        );
    }

    return (
        <div className="chart-container">
            <div style={{ height }}>
                <ResponsiveContainer width="100%" height="100%">
                    <LineChart data={data}>
                        <CartesianGrid strokeDasharray="3 3" stroke="var(--border-color)" vertical={false} />
                        <XAxis hide />
                        <YAxis domain={domain} stroke="var(--text-muted)" fontSize={10} tickFormatter={(val) => Math.round(val)} width={30} />
                        <Tooltip content={<CustomTooltip />} isAnimationActive={false} />
                        {lines.map((line, idx) => (
                            <Line key={idx} type="monotone" dataKey={line.dataKey} stroke={line.color} dot={false} strokeWidth={2} isAnimationActive={false} strokeDasharray={line.dash} />
                        ))}
                    </LineChart>
                </ResponsiveContainer>
            </div>
        </div>
    );
};

export const Charts = ({ dataHistory }) => {
    return (
        <div className="charts-wrapper custom-scrollbar">
            <div className="chart-header">
                <h3 style={{ fontSize: "1.125rem", fontWeight: 600 }}>Wykresy w czasie rzeczywistym</h3>
                <span style={{ fontSize: "0.75rem", fontFamily: "var(--font-mono)", color: "var(--text-muted)" }}>Ostatnie {dataHistory.length} próbek</span>
            </div>

            {/* Main Distance Chart */}
            <div style={{ marginBottom: "4px" }}>
                <span className="chart-title">Odległość i Setpoint</span>
            </div>
            <div style={{ display: "flex", gap: "8px" }}>
                <div style={{ flex: 7, minWidth: 0 }}>
                    <CommonChart
                        data={dataHistory}
                        height={250}
                        domain={[0, 300]}
                        lines={[
                            { dataKey: "setpoint", color: "var(--chart-setpoint)", dash: "4 4" },
                            { dataKey: "distance", color: "var(--chart-dist)", dash: "2 2" },
                            { dataKey: "filtered", color: "var(--chart-filter)" },
                        ]}
                    />
                </div>
                <div style={{ flex: 3, minWidth: 0 }}>
                    <CommonChart
                        data={dataHistory.slice(-30)}
                        height={250}
                        domain={["auto", "auto"]}
                        lines={[
                            { dataKey: "setpoint", color: "var(--chart-setpoint)", dash: "4 4" },
                            { dataKey: "distance", color: "var(--chart-dist)", dash: "2 2" },
                            { dataKey: "filtered", color: "var(--chart-filter)" },
                        ]}
                    />
                </div>
            </div>

            {/* Error Chart */}
            <div style={{ marginBottom: "4px" }}>
                <span className="chart-title">Uchyb Regulacji</span>
            </div>
            <CommonChart data={dataHistory} height={150} lines={[{ dataKey: "error", color: "var(--chart-error)" }]} />

            {/* Control Signal Chart */}
            <div style={{ marginBottom: "4px" }}>
                <span className="chart-title">Sygnał Sterujący</span>
            </div>
            <CommonChart data={dataHistory} height={150} lines={[{ dataKey: "control", color: "var(--chart-ctrl)" }]} />
        </div>
    );
};
