import React from "react";

export const MetricCard = ({ label, value, unit, color }) => {
    return (
        <div className="metric-card">
            <div className="metric-label">{label}</div>
            <div className="metric-value-row">
                <span className="metric-value" style={{ color }}>
                    {value}
                </span>
                <span className="metric-unit">{unit}</span>
            </div>
        </div>
    );
};

export const Metrics = ({ data }) => {
    return (
        <div className="metrics-grid">
            <MetricCard label="Dystans (Raw)" value={data.distance.toFixed(0)} unit="mm" color="var(--chart-dist)" />
            <MetricCard label="Dystans (Filtrowany)" value={data.filtered.toFixed(0)} unit="mm" color="var(--chart-filter)" />
            <MetricCard label="Uchyb (Błąd)" value={data.error.toFixed(1)} unit="mm" color="var(--chart-error)" />
            <MetricCard label="Częstotliwość" value={data.freq} unit="Hz" color="var(--text-secondary)" />
        </div>
    );
};
