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

        // Weź dane z ostatniej sekundy (zakładając ~50-100Hz, bierzemy ostatnie 100 próbek)
        const recentData = dataHistory.slice(-100);

        // Średni uchyb procentowy
        const avgError = recentData.reduce((sum, d) => sum + Math.abs(d.error), 0) / recentData.length;
        const avgSetpoint = recentData.reduce((sum, d) => sum + d.setpoint, 0) / recentData.length;
        const avgErrorPercent = avgSetpoint > 0 ? (avgError / avgSetpoint) * 100 : 0;

        // Stabilność (odchylenie standardowe błędu)
        const errorVariance = recentData.reduce((sum, d) => sum + Math.pow(d.error - avgError, 2), 0) / recentData.length;
        const stability = 100 - Math.min(Math.sqrt(errorVariance), 100);

        // Min/Max dystans
        const distances = recentData.map((d) => d.filtered);
        const minDistance = Math.min(...distances);
        const maxDistance = Math.max(...distances);

        return {
            avgErrorPercent: avgErrorPercent.toFixed(1),
            stability: stability.toFixed(0),
            minDistance: minDistance.toFixed(0),
            maxDistance: maxDistance.toFixed(0),
        };
    }, [dataHistory]);

    return (
        <div className="metrics-grid">
            {/* First row - podstawowe metryki */}
            <MetricCard label="Dystans (Raw)" value={data.distance.toFixed(0)} unit="mm" color="var(--chart-dist)" />
            <MetricCard label="Dystans (Filtrowany)" value={data.filtered.toFixed(0)} unit="mm" color="var(--chart-filter)" />
            <MetricCard label="Uchyb (Błąd)" value={data.error.toFixed(1)} unit="mm" color="var(--chart-error)" />
            <MetricCard label="Częstotliwość" value={data.freq} unit="Hz" color="var(--text-secondary)" />

            {/* Second row - zaawansowane metryki */}
            <MetricCard label="Średni Uchyb" value={computedMetrics.avgErrorPercent} unit="%" color="var(--warning)" />
            <MetricCard label="Stabilność" value={computedMetrics.stability} unit="%" color="var(--success)" />
            <MetricCard label="Min (1s)" value={computedMetrics.minDistance} unit="mm" color="var(--primary)" />
            <MetricCard label="Max (1s)" value={computedMetrics.maxDistance} unit="mm" color="var(--primary)" />
        </div>
    );
};
