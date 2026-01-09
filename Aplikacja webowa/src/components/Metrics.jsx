import React, { useMemo } from "react";

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

export const Metrics = ({ data, dataHistory }) => {
    // Oblicz dodatkowe metryki z ostatniej sekundy danych
    const computedMetrics = useMemo(() => {
        if (!dataHistory || dataHistory.length < 2) {
            return {
                avgErrorPercent: 0,
                stability: 0,
                minDistance: 0,
                maxDistance: 0,
            };
        }

        const recentData = dataHistory.slice(-25);

        const avgError = recentData.reduce((sum, d) => sum + Math.abs(d.error), 0) / recentData.length;
        const avgSetpoint = recentData.reduce((sum, d) => sum + d.setpoint, 0) / recentData.length;
        const avgErrorPercent = avgSetpoint > 0 ? (avgError / avgSetpoint) * 100 : 0;

        const errorVariance = recentData.reduce((sum, d) => sum + Math.pow(d.error - avgError, 2), 0) / recentData.length;
        const stability = 100 - Math.min(Math.sqrt(errorVariance), 100);

        const distances = recentData.map((d) => d.filtered);
        const minDistance = Math.min(...distances);
        const maxDistance = Math.max(...distances);

        return {
            avgErrorPercent: avgErrorPercent.toFixed(1),
            stdDev: Math.sqrt(errorVariance).toFixed(1),
        };
    }, [dataHistory]);

    return (
        <div className="metrics-grid">
            {/* First row - Główne Pomiary */}
            <MetricCard label="Dystans (Raw)" value={data.distance.toFixed(0)} unit="mm" color="var(--chart-dist)" />
            <MetricCard label="Dystans (Filtrowany)" value={data.filtered.toFixed(0)} unit="mm" color="var(--chart-filter)" />
            <MetricCard label="Uchyb (Błąd)" value={data.error.toFixed(1)} unit="mm" color="var(--chart-error)" />
            <MetricCard label="Średni Uchyb (25 próbek)" value={computedMetrics.avgErrorPercent} unit="%" color="var(--warning)" />

            {/* Second row - Diagnostyka */}
            <MetricCard label="Częstotliwość" value={data.freq} unit="Hz" color="var(--text-secondary)" />
            <MetricCard label="Kąt Serwa" value={data.control.toFixed(0)} unit="°" color="#3498db" />
            <MetricCard label="Odchylenie Std." value={computedMetrics.stdDev} unit="mm" color="#9b59b6" />
            <MetricCard label="Cel (Setpoint)" value={data.setpoint.toFixed(0)} unit="mm" color="var(--chart-setpoint)" />
        </div>
    );
};
