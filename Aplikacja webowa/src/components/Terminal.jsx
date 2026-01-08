import React, { useEffect, useMemo, useRef } from "react";

export const Terminal = ({ logs }) => {
    const txBodyRef = useRef(null);
    const rxBodyRef = useRef(null);

    // Logs are already split by useSerial for performance
    const { tx: txLogs, rx: rxLogs } = logs;

    // Auto-scroll TX column
    useEffect(() => {
        if (txBodyRef.current) {
            txBodyRef.current.scrollTop = txBodyRef.current.scrollHeight;
        }
    }, [txLogs]);

    // Auto-scroll RX column
    useEffect(() => {
        if (rxBodyRef.current) {
            rxBodyRef.current.scrollTop = rxBodyRef.current.scrollHeight;
        }
    }, [rxLogs]);

    return (
        <div className="terminal-container">
            <div className="terminal-header">
                <span className="terminal-title">Logi Komunikacji</span>
            </div>
            <div className="terminal-columns">
                {/* Left Column: TX (Sent) */}
                <div className="terminal-column">
                    <div className="terminal-column-header">
                        <span>📤 Wysłane (TX)</span>
                        <span className="log-count">{txLogs.length}</span>
                    </div>
                    <div className="terminal-body custom-scrollbar" ref={txBodyRef}>
                        {txLogs.length === 0 && <div style={{ color: "var(--text-muted)", fontStyle: "italic", padding: "0.5rem" }}>Brak wysłanych ramek...</div>}
                        {txLogs.map((log, i) => (
                            <div key={i} className="log-entry">
                                <span className="timestamp">[{log.time}]</span>
                                <span className="log-tx">{log.msg}</span>
                            </div>
                        ))}
                    </div>
                </div>

                {/* Right Column: RX (Received) */}
                <div className="terminal-column">
                    <div className="terminal-column-header">
                        <span>📥 Odebrane (RX)</span>
                        <span className="log-count">{rxLogs.length}</span>
                    </div>
                    <div className="terminal-body custom-scrollbar" ref={rxBodyRef}>
                        {rxLogs.length === 0 && <div style={{ color: "var(--text-muted)", fontStyle: "italic", padding: "0.5rem" }}>Brak odebranych ramek...</div>}
                        {rxLogs.map((log, i) => (
                            <div key={i} className="log-entry">
                                <span className="timestamp">[{log.time}]</span>
                                <span className={log.type === "error" ? "log-error" : log.type === "success" ? "log-success" : ""}>{log.msg}</span>
                            </div>
                        ))}
                    </div>
                </div>
            </div>
        </div>
    );
};
